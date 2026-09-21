# -*- coding: utf-8 -*-
"""
北云（UG016 组合导航接收机）GNSS 热启动（断天线重接）收敛分析
=====================================================================
由 BY_coldstart.py 改造：冷启动 = 单次上电收敛；热启动 = 一份日志内含多次
"断开天线 -> 重新接上天线"循环，需自动切割周期并统计每次从恢复锁定到
单点/浮点/固定解的耗时与状态。

测试过程（用户定义）：户外连续录制中直接断开天线再接上，卫星数归零后重新搜星，
直到重新获得定位（单点->浮点->固定）。

起点事件（t=0）的选取（已核实，非猜测）：
  * 候选1：ANTENNASTATUS（华测 3.2.1）可标记天线开路/接通，但本批数据未录制 -> 不可用。
  * 采用：TRACKSTATA 任一通道 locktime 由 0 变为 >0（信号重新锁定）。
    依据 UG016 4.2.26 字段12"locktime=连续跟踪秒数(无周跳)"；locktime>0 表示码+载波已锁，
    早于该星计入 #SVs、更早于定位解输出，比"第一颗星(#SVs>0)"更早且对搜星慢的机器更公平。
  * 双机取早：北云/华测同时断、同时上，同一测试周期取两台中更早的 locktime 起点作为 t=0。

报文口径（UG016，by_manual/UG016.md）：
  * 定位主报文  BESTGNSSPOSA（4.2.2）：定位类型表4-2、卫星 #SVs/#solnSVs/#solnMultiSVs(字段15/16/18)
  * 锁定判定    TRACKSTATA（4.2.26）：#chans(字段5)、ch-tr-status(字段8)、C/No(字段11)、locktime(字段12)
  * 差分可用    Stn ID 非空且 != "0"
  * CRC         "#"=32位CRC，"$"=NMEA异或；$KSXT 为私有8位CRC32（按异或判失败属正常）

输出（reports 目录，每次运行覆盖）：
  report.html / report.md / img_周期时刻.png / img_阶段耗时.png / img_卫星数量时间轴.png / summary.json
运行：python gnss_warmstart_hmi.py（HMI 选北云）；或
      python BY_warmstart_.py <输出目录> <北云日志> [华测日志(可选,用于双机取早)]
"""
import os
import sys
import io
import json
import base64
import collections
import statistics
import webbrowser

ROOT = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(ROOT, 'reports')
TAG = '北云GNSS热启动分析报告'
MANUAL = '北云科技《UG016 数据通信接口协议》'
PRODUCT = '北云'

if sys.stdout.encoding and sys.stdout.encoding.lower() not in ('utf-8', 'utf8'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# 定位类型分组（UG016 表4-2）：1=单点类 / 2=浮点类 / 3=固定类，其余 0=无解
CAT1 = {'SINGLE', 'PSRDIFF', 'WAAS', 'INS_PSRSP', 'INS_PSRDIFF', 'INS_SBAS'}
CAT2 = {'L1_FLOAT', 'IONOFREE_FLOAT', 'NARROW_FLOAT', 'FLOATCONV', 'WIDELANE',
        'NARROWLANE', 'INS_RTKFLOAT'}
CAT3 = {'L1_INT', 'WIDE_INT', 'NARROW_INT', 'INS_RTKFIXED', 'RTK_DIRECT_INS', 'PPP', 'INS_PPP'}
TS_CODE = {'UNKNOWN': 0, 'COARSE': 1, 'FINESTEERING': 2}
PT_CN = {'NONE': '无解', 'SINGLE': '单点', 'NARROW_FLOAT': 'RTK浮点', 'NARROW_INT': 'RTK固定'}
CAT_CN = ['无解', '单点', 'RTK浮点', 'RTK固定']

# 周期切割参数
MIN_OUTAGE_S = 8.0      # 断线最短持续（秒），短于此判为树荫/抖动
MERGE_GAP_S = 8.0       # 相邻两段 0 锁定间隔小于此值合并（断线中的短暂抖动）
PAIR_TOL_S = 30.0       # 双机断线起点相差在此范围内才配对为同一次

# ----------------------------------------------------------------------
# CRC 与报文读取
# ----------------------------------------------------------------------

def crc32b(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc & 0xFFFFFFFF


def iter_msgs(path):
    """逐条产出 CRC 校验通过的报文 (报文名, 报头字段, 数据字段)。'#'=CRC32，'$'=NMEA异或。"""
    raw = open(path, 'rb').read()
    bad = 0
    for line in raw.split(b'\n'):
        line = line.rstrip(b'\r')
        if not line or line[0] not in (0x23, 0x24):
            continue
        star = line.rfind(b'*')
        if star <= 0:
            bad += 1
            continue
        body = line[1:star]
        try:
            if line[0] == 0x23:
                ok = crc32b(body) == int(line[star + 1:star + 9], 16)
            else:
                c = 0
                for bb in body:
                    c ^= bb
                ok = c == int(line[star + 1:star + 3], 16)
        except ValueError:
            ok = False
        if not ok:
            bad += 1
            continue
        txt = body.decode('ascii', 'ignore')
        if ';' in txt:
            h, b = txt.split(';', 1)
            yield h.split(',')[0].split(',')[0], h.split(','), b.split(','), 'hash'
        else:
            yield txt.split(',')[0], None, txt.split(','), 'nmea'
    yield '__BAD__', None, bad, None


def htime_s(hf):
    """北云报头 GPS 时间（字段5=周, 字段6=周内秒）-> 绝对秒。"""
    try:
        return int(hf[5]) * 604800 + float(hf[6])
    except Exception:
        return None

# ----------------------------------------------------------------------
# 动态选择含 locktime 的最高频报文
# ----------------------------------------------------------------------

def parse_trackstat(bf):
    """TRACKSTATA -> 该帧有效锁定通道数（locktime>0 且 C/No>0 且非NA）。"""
    try:
        nch = int(bf[3])
    except Exception:
        return 0
    recs = bf[4:]
    per = 10
    locked = 0
    for i in range(nch):
        rec = recs[i * per:(i + 1) * per]
        if len(rec) < per:
            continue
        try:
            cno = float(rec[5]); lt = float(rec[6]); rj = rec[7]
        except Exception:
            continue
        if lt > 0 and cno > 0 and rj != 'NA':
            locked += 1
    return locked


def build_lock_series(path):
    """返回 (times, locked_counts, 来源报文名, 实测周期s)。北云当前为 TRACKSTATA。
    动态选择：在所有含 locktime 的报文中取实测频率最高者。"""
    cands = collections.defaultdict(list)   # name -> [(t, locked)]
    for name, hf, bf, kind in iter_msgs(path):
        if name == '__BAD__':
            continue
        if kind != 'hash' or hf is None:
            continue
        t = htime_s(hf)
        if t is None:
            continue
        if name.startswith('TRACKSTAT'):
            cands['TRACKSTATA'].append((t, parse_trackstat(bf)))
    best, best_period = None, None
    for name, seq in cands.items():
        if len(seq) < 3:
            continue
        dts = sorted(seq[k][0] - seq[k - 1][0] for k in range(1, len(seq)))
        period = dts[len(dts) // 2]
        if best_period is None or period < best_period:
            best, best_period = (name, seq), period
    if best is None:
        raise SystemExit('未找到含 locktime 的报文（TRACKSTATA）')
    name, seq = best
    times = [x[0] for x in seq]
    locked = [x[1] for x in seq]
    return times, locked, name, best_period

# ----------------------------------------------------------------------
# 周期切割：0 锁定段 -> 各热启动周期起点
# ----------------------------------------------------------------------

def cut_cycles(times, locked, min_outage=MIN_OUTAGE_S, merge_gap=MERGE_GAP_S):
    """返回周期列表：[{outage_start, outage_end, t_lock(=本机起点), outage_dur}]。
    一段连续 locked==0 视为一次断线；t_lock = 该段之后第一个 locked>0 的时刻。"""
    n = len(times)
    segs = []
    i = 0
    while i < n:
        if locked[i] == 0:
            j = i
            while j + 1 < n and locked[j + 1] == 0:
                j += 1
            segs.append([i, j])
            i = j + 1
        else:
            i += 1
    # 合并相邻近段
    merged = []
    for a, b in segs:
        if merged and times[a] - times[merged[-1][1]] < merge_gap:
            merged[-1][1] = b
        else:
            merged.append([a, b])
    cycles = []
    for a, b in merged:
        dur = times[b] - times[a]
        if dur < min_outage:
            continue
        nxt = b + 1
        if nxt >= n:
            continue
        cycles.append(dict(outage_start=times[a], outage_end=times[b],
                           t_lock=times[nxt], outage_dur=round(dur, 2),
                           lock_idx=nxt))
    # 事件窗口必须在下一周期断线开始前截止，避免把后续周期状态混入当前周期。
    for i, c in enumerate(cycles):
        c['next_outage_start'] = cycles[i + 1]['outage_start'] if i + 1 < len(cycles) else None
    return cycles

# ----------------------------------------------------------------------
# 主定位序列（BESTGNSSPOSA）
# ----------------------------------------------------------------------

def build_pos_series(path):
    rows = []
    counts = collections.Counter()
    other = collections.Counter()
    bad = 0
    for name, hf, bf, kind in iter_msgs(path):
        if name == '__BAD__':
            bad = hf if hf is not None else 0
            continue
        if kind == 'hash':
            counts[name] += 1
        else:
            other[name] += 1
        if not name.startswith('BESTGNSSPOS') or hf is None:
            continue
        t = htime_s(hf)
        if t is None or len(bf) < 21:
            continue
        try:
            rows.append(dict(t=t, ts=hf[4], sol=bf[0], pt=bf[1],
                             stn=bf[10].strip('"'), age=float(bf[11]),
                             svs=int(bf[13]), soln=int(bf[14]), multi=int(bf[16])))
        except Exception:
            continue
    return rows, counts, other, bad


def cat_of(pt):
    return 3 if pt in CAT3 else 2 if pt in CAT2 else 1 if pt in CAT1 else 0

# ----------------------------------------------------------------------
# 跨设备配对取早
# ----------------------------------------------------------------------

def pair_earliest(my_cycles, other_cycles, tol=PAIR_TOL_S):
    """按断线起点配对两台设备的周期；同一次取两台 t_lock 中更早者作为 t0。
    返回 {本机cycle索引: dict(t0, other_t_lock, paired)}。"""
    res = {}
    used = set()
    for ci, c in enumerate(my_cycles):
        best_j, best_dt = None, None
        for j, oc in enumerate(other_cycles):
            if j in used:
                continue
            dt = abs(oc['outage_start'] - c['outage_start'])
            if dt <= tol and (best_dt is None or dt < best_dt):
                best_j, best_dt = j, dt
        if best_j is not None:
            used.add(best_j)
            oc = other_cycles[best_j]
            t0 = min(c['t_lock'], oc['t_lock'])
            res[ci] = dict(t0=t0, other_t_lock=oc['t_lock'], paired=True)
        else:
            res[ci] = dict(t0=c['t_lock'], other_t_lock=None, paired=False)
    return res

# ----------------------------------------------------------------------
# 单周期分析
# ----------------------------------------------------------------------

def analyze_cycle(cyc, t0, rows, after=180.0):
    """当前周期内的关键时刻；截止于本设备下一次断线开始。"""
    cycle_end = cyc.get('next_outage_start')
    end = min(t0 + after, cycle_end) if cycle_end is not None else t0 + after
    win = [r for r in rows if t0 - 0.05 <= r['t'] <= end]
    valid = [r for r in win if r['sol'] == 'SOL_COMPUTED']
    def first(pred):
        for r in valid:
            if pred(r):
                return round(r['t'] - t0, 1)
        return None
    f = dict(
        single=first(lambda r: cat_of(r['pt']) == 1),
        float=first(lambda r: cat_of(r['pt']) == 2),
        fixed=first(lambda r: cat_of(r['pt']) == 3),
        stn=first(lambda r: r['stn'] not in ('', '0')),
        svs10=first(lambda r: r['svs'] >= 10),
        svs20=first(lambda r: r['svs'] >= 20),
    )
    # 首固定保持 5s
    fixed_hold = None
    for i, r in enumerate(valid):
        if cat_of(r['pt']) == 3:
            hold = all(cat_of(x['pt']) == 3 for x in valid[i:] if x['t'] <= r['t'] + 5.0)
            if hold:
                fixed_hold = round(r['t'] - t0, 1)
                break
    def dur(a, b):
        return None if (a is None or b is None or b < a) else round(b - a, 1)
    stages = dict(lock_to_single=f['single'], single_to_float=dur(f['single'], f['float']),
                  float_to_fixed=dur(f['float'], f['fixed']), lock_to_fixed=f['fixed'])
    series = dict(t=[round(r['t'] - t0, 1) for r in win], svs=[r['svs'] for r in win],
                  soln=[r['soln'] for r in win], multi=[r['multi'] for r in win],
                  cats=[cat_of(r['pt']) for r in win])
    return dict(first=f, stages=stages, fixed_hold=fixed_hold, series=series,
                n=len(win), valid_n=len(valid), window_end=round(end - t0, 1))

# ----------------------------------------------------------------------
# 主流程
# ----------------------------------------------------------------------

def build(by_path, hc_path=None):
    name = os.path.basename(by_path)
    lt_t, lt_c, lt_src, lt_period = build_lock_series(by_path)
    my_cycles = cut_cycles(lt_t, lt_c)
    pair = {}
    other_src = None
    if hc_path and os.path.isfile(hc_path):
        try:
            import importlib.util
            spec = importlib.util.spec_from_file_location(
                'HUACE_warmstart', os.path.join(ROOT, 'HUACE_warmstart.py'))
            HC = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(HC)
            ot, oc, osrc, oper = HC.build_lock_series(hc_path)
            other_cycles = HC.cut_cycles(ot, oc)
            pair = pair_earliest(my_cycles, other_cycles)
            other_src = osrc
        except Exception as e:
            print('华测配对失败（改用本机起点）:', e)
    rows, counts, other, bad = build_pos_series(by_path)
    cycles = []
    for ci, c in enumerate(my_cycles):
        t0 = pair.get(ci, {}).get('t0', c['t_lock'])
        info = analyze_cycle(c, t0, rows)
        cycles.append(dict(idx=ci + 1, outage_start=round(c['outage_start'] - lt_t[0], 1),
                           outage_dur=c['outage_dur'], t_lock=round(c['t_lock'] - lt_t[0], 1),
                           other_t_lock=(None if not pair.get(ci, {}).get('paired')
                                         else round(pair[ci]['other_t_lock'] - lt_t[0], 1)),
                           t0=t0, t0_abs=t0, paired=pair.get(ci, {}).get('paired', False),
                           **info))
    return dict(file=name, product=PRODUCT, lock_src=lt_src, lock_rate=round(1.0 / lt_period, 1),
                other_lock_src=other_src, n_cycles=len(cycles), cycles=cycles,
                counts=dict(counts), other=dict(other), crc_bad=bad,
                span=round(rows[-1]['t'] - rows[0]['t'], 1) if rows else 0)


def agg(cycles):
    """跨周期统计。"""
    def stats(key, getter):
        vals = [getter(c) for c in cycles]
        vals = [v for v in vals if v is not None]
        if not vals:
            return None
        return dict(min=round(min(vals), 1), mean=round(statistics.mean(vals), 1),
                    max=round(max(vals), 1), n=len(vals))
    return dict(
        outage=stats('o', lambda c: c['outage_dur']),
        single=stats('s', lambda c: c['first']['single']),
        float=stats('f', lambda c: c['first']['float']),
        fixed=stats('x', lambda c: c['first']['fixed']),
        fixed_hold=stats('h', lambda c: c['fixed_hold']),
        single_to_float=stats('sf', lambda c: c['stages']['single_to_float']),
        float_to_fixed=stats('ff', lambda c: c['stages']['float_to_fixed']),
    )


def short_label(name, width=16):
    base = name.replace('.log', '').replace('.dat', '')
    return base if len(base) <= width else base[:width] + '\n' + base[width:width * 2 - 3] + '…'
# ----------------------------------------------------------------------
# 绘图
# ----------------------------------------------------------------------

def setup_mpl():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    for fam in ('Microsoft YaHei', 'SimHei', 'Noto Sans CJK SC', 'WenQuanYi Zen Hei'):
        try:
            from matplotlib import font_manager
            if any(fam.lower() in f.name.lower() for f in font_manager.fontManager.ttflist):
                plt.rcParams['font.sans-serif'] = [fam] + plt.rcParams.get('font.sans-serif', [])
                break
        except Exception:
            pass
    plt.rcParams['axes.unicode_minus'] = False
    return plt


def make_images(data, out_dir):
    plt = setup_mpl()
    paths = {}
    cyc = data['cycles']
    xs = list(range(len(cyc)))
    labs = ['#%d' % c['idx'] for c in cyc]

    # 图1：各周期关键时刻
    fig, ax = plt.subplots(figsize=(max(11, len(cyc) * 0.9), 4.8))
    marks = [('single', '首单点/伪距类', '#3b82f6', 's'),
             ('float', '首浮点', '#f59e0b', '^'),
             ('fixed', '首固定', '#16a34a', 'D')]
    for key, lab, col, mk in marks:
        ys = [c['first'][key] if c['first'][key] is not None else float('nan') for c in cyc]
        ax.scatter(xs, ys, marker=mk, s=70, color=col, label=lab, zorder=3)
        for x, y in zip(xs, ys):
            if y == y:
                ax.annotate('%.1f' % y, (x, y), textcoords='offset points', xytext=(4, 5),
                            fontsize=7.5, color=col)
    ax.set_xticks(xs); ax.set_xticklabels(labs, fontsize=9)
    ax.set_xlabel('热启动周期（断天线重接次序）'); ax.set_ylabel('自起点(locktime)耗时（秒）')
    ax.set_title('%s 各热启动周期关键时刻（t=0 为双机更早的 locktime 起点）' % PRODUCT, fontsize=13)
    ax.grid(axis='y', ls='--', alpha=0.4); ax.legend(loc='upper left', ncols=3, fontsize=9)
    fig.tight_layout()
    paths['keytime'] = os.path.join(out_dir, 'img_周期时刻.png')
    fig.savefig(paths['keytime'], dpi=150); plt.close(fig)

    # 图2：阶段耗时分解（堆叠）
    fig, ax = plt.subplots(figsize=(max(12, len(cyc) * 0.9), 5.0))
    stage_def = [('lock_to_single', '锁定→单点', '#f59e0b'),
                 ('single_to_float', '单点→浮点', '#3b82f6'),
                 ('float_to_fixed', '浮点→固定', '#16a34a')]
    for i, c in enumerate(cyc):
        left = 0.0
        for key, lab, col in stage_def:
            v = c['stages'][key] or 0
            if v > 0:
                ax.barh(i, v, left=left, color=col, edgecolor='white', height=0.55)
                if v >= 8:
                    ax.text(left + v / 2, i, '%.1f' % v, ha='center', va='center', color='white', fontsize=8)
                left += v
        if c['first']['fixed'] is not None:
            ax.text(left + 2, i, '固定 %.1fs' % c['first']['fixed'], va='center', fontsize=8, color='#111827')
    ax.set_yticks(xs); ax.set_yticklabels(labs, fontsize=9); ax.invert_yaxis()
    ax.set_xlabel('时间（秒）')
    ax.set_title('%s 热启动阶段耗时分解（锁定→单点→浮点→固定）' % PRODUCT, fontsize=13)
    ax.grid(axis='x', ls='--', alpha=0.4)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for _, _, c in stage_def]
    ax.legend(handles, [l for _, l, _ in stage_def], loc='lower right', ncols=3, fontsize=9)
    fig.tight_layout()
    paths['stages'] = os.path.join(out_dir, 'img_阶段耗时.png')
    fig.savefig(paths['stages'], dpi=150); plt.close(fig)

    # 图3：各周期卫星数恢复曲线
    nrow = len(cyc)
    fig, axes = plt.subplots(nrow, 1, figsize=(13, 2.4 * nrow), squeeze=False)
    for i, c in enumerate(cyc):
        ax = axes[i][0]
        s = c['series']
        ax.plot(s['t'], s['svs'], color='#2563eb', lw=1.2, label='跟踪星 #SVs', drawstyle='steps-post')
        ax.plot(s['t'], s['soln'], color='#16a34a', lw=1.0, label='解算星 #solnSVs', drawstyle='steps-post')
        ax.set_ylim(0, max(s['svs'] + [5]) * 1.3)
        for key, col, lab in [('single', '#3b82f6', '单点'), ('float', '#f59e0b', '浮点'), ('fixed', '#16a34a', '固定')]:
            v = c['first'][key]
            if v is not None:
                ax.axvline(v, color=col, ls='--', lw=0.9)
                ax.text(v, ax.get_ylim()[1] * 0.92, lab, color=col, fontsize=7, rotation=90, va='top')
        ax.set_title('周期 #%d（断线 %.0fs，窗口截止 %.1fs）' % (c['idx'], c['outage_dur'], c['window_end']), fontsize=9)
        ax.set_xlim(0, max(1.0, max(s['t']) if s['t'] else 60))
        if i == 0:
            ax.legend(loc='lower right', fontsize=7.5)
        ax.grid(ls='--', alpha=0.3)
        ax.tick_params(labelsize=7.5)
    fig.suptitle('%s 各周期卫星数量恢复曲线（t=0 为 locktime 起点）' % PRODUCT, fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.99])
    paths['svs'] = os.path.join(out_dir, 'img_卫星数量时间轴.png')
    fig.savefig(paths['svs'], dpi=150); plt.close(fig)
    return paths


def img64(path):
    return 'data:image/png;base64,' + base64.b64encode(open(path, 'rb').read()).decode('ascii')


def V(v, u='s'):
    return '—' if v is None else ('%.1f%s' % (v, u))


def S(st):
    if st is None:
        return '—'
    return '%.1f / %.1f / %.1f (n=%d)' % (st['min'], st['mean'], st['max'], st['n'])
# ----------------------------------------------------------------------
# HTML / MD 报告
# ----------------------------------------------------------------------

CSS = """
 *{box-sizing:border-box}
 body{font-family:"Microsoft YaHei","Segoe UI",sans-serif;margin:0;background:#f5f7fa;color:#1f2937}
 .wrap{max-width:1360px;margin:0 auto;padding:24px}
 h1{font-size:22px;margin:0 0 4px}
 h2{font-size:16px;margin:28px 0 10px;color:#111827;border-left:4px solid #2563eb;padding-left:8px}
 .sub{color:#6b7280;font-size:13px;margin-bottom:14px}
 .card{background:#fff;border:1px solid #e5e7eb;border-radius:8px;padding:14px;margin-bottom:14px}
 table{border-collapse:collapse;width:100%;font-size:12.5px}
 th,td{border:1px solid #e5e7eb;padding:5px 7px;text-align:left}
 th{background:#f3f4f6;font-weight:600}
 .mono{font-family:Consolas,monospace;font-size:12px}
 .note{font-size:12px;color:#6b7280;margin-top:6px}
 .imgcard{background:#fff;border:1px solid #e5e7eb;border-radius:8px;padding:10px;margin-bottom:12px}
 .imgcard img{width:100%;display:block}
 .ok{color:#16a34a;font-weight:600}
"""

HTML = """<!DOCTYPE html>
<html lang="zh-CN"><head><meta charset="UTF-8">
<title>__TAG__</title><style>__CSS__</style></head>
<body><div class="wrap">
<h1>__PRODUCT__ GNSS 热启动（断天线重接）收敛分析</h1>
<div class="sub">数据源：__SRC__｜口径依据：__MANUAL__｜起点事件：locktime 0→&gt;0（__LOCKSRC__ __LOCKRATE__Hz）__PAIRNOTE__｜周期切割：0 锁定 ≥ __MINOUT__s 记为一次断线</div>

<h2>一、数据完整性与周期切割</h2>
<div class="card"><table>
<tr><th>文件</th><th>时长</th><th>锁定源报文</th><th>锁定源频率</th><th>CRC失败行</th><th>检出热启动周期数</th></tr>
<tr><td class='mono'>__FILE__</td><td>__SPAN__s</td><td>__LOCKSRC__</td><td>__LOCKRATE__Hz</td><td>__CRCBAD__</td><td><b>__NCYC__</b></td></tr>
</table>
<div class="note">锁定源 = 含 locktime 字段且实测频率最高的报文（动态选择，__LOCKSRC__）。定位主报文：__MAINMSG__。周期起点取"两台设备中更早的 locktime 翻正时刻"；未配对成功的周期退回本机 locktime 起点。</div></div>

<h2>二、各热启动周期关键时刻（自起点秒）</h2>
<div class="card"><table>
<tr><th>周期</th><th>断线时长</th><th>本机起点</th><th>对端起点</th><th>取早t0</th><th>首单点/伪距类</th><th>首浮点</th><th>首固定</th><th>固定保持5s</th><th>差分可用</th><th>星数≥10</th><th>星数≥20</th></tr>
__CYCTABLE__
</table></div>
<div class="imgcard"><img src="__IMG1__"></div>

<h2>三、阶段耗时分解</h2>
<div class="imgcard"><img src="__IMG2__"></div>

<h2>四、跨周期统计</h2>
<div class="card"><table>
<tr><th>指标</th><th>min / 均值 / max (n)</th></tr>
<tr><td>断线时长(s)</td><td>__AGG_OUT__</td></tr>
<tr><td>起点→首单点/伪距类(s)</td><td>__AGG_S__</td></tr>
<tr><td>起点→首浮点(s)</td><td>__AGG_F__</td></tr>
<tr><td>起点→首固定(s)</td><td>__AGG_X__</td></tr>
<tr><td>起点→固定保持5s(s)</td><td>__AGG_H__</td></tr>
<tr><td>单点→浮点(s)</td><td>__AGG_SF__</td></tr>
<tr><td>浮点→固定(s)</td><td>__AGG_FF__</td></tr>
</table></div>

<h2>五、各周期卫星数量恢复曲线</h2>
<div class="imgcard"><img src="__IMG3__"></div>

<h2>附录：口径与手册条款对应</h2>
<div class="card"><table>
<tr><th>报告字段</th><th>数据来源</th><th>手册条款</th><th>说明</th></tr>
<tr><td>热启动起点(t=0)</td><td>__LOCKSRC__ 任一通道 locktime 0→&gt;0</td><td>__LOCKCLAUSE__</td><td>locktime=连续跟踪秒数(无周跳)；&gt;0 表示码+载波已锁，早于计入 #SVs 与定位解；双机取更早者</td></tr>
<tr><td>首单点/浮点/固定</td><td>定位报文 Pos Type 首次 SINGLE/NARROW_FLOAT/NARROW_INT</td><td>__PTCLAUSE__</td><td>定位类型枚举</td></tr>
<tr><td>跟踪星/解算星/多频</td><td>#SVs/#solnSVs/#solnMultiSVs</td><td>__SVCLAUSE__</td><td>卫星计数字段</td></tr>
<tr><td>差分可用</td><td>Stn ID 非空且!="0"</td><td>__STNCLAUSE__</td><td>基准站ID</td></tr>
</table>
<div class="note">起点不含"天线接上的物理瞬间"：该瞬间在日志中无任何报文记录（需 ANTENNASTATUS 或 EVENT 硬件标记，本批未录制）。locktime 起点是"信号重新锁定"的可测最早点，已避开"搜星慢机器吃亏"问题——搜星耗时被如实计入锁定前的等待。</div></div>
</div></body></html>
"""


def build_outputs(data, imgs):
    a = agg(data['cycles'])
    rows = []
    for c in data['cycles']:
        f = c['first']
        rows.append(
            "<tr><td>#%d</td><td>%.0fs</td><td>%.1f</td><td>%s</td><td>%.1f</td>"
            "<td>%s</td><td>%s</td><td class='ok'>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
                c['idx'], c['outage_dur'], c['t_lock'],
                ('—' if c['other_t_lock'] is None else '%.1f' % c['other_t_lock']),
                round(c['t0'] - (c['t0_abs'] - c['t_lock']) if False else c['t_lock'], 1),
                V(f['single']), V(f['float']), V(f['fixed']), V(c['fixed_hold']),
                V(f['stn']), V(f['svs10']), V(f['svs20'])))
    cyctable = ''.join(rows)
    pairnote = '（已与对端设备取早配对）' if data.get('other_lock_src') else '（单设备，未取早）'
    html = (HTML
            .replace('__TAG__', TAG).replace('__CSS__', CSS).replace('__PRODUCT__', PRODUCT)
            .replace('__SRC__', data['file']).replace('__MANUAL__', MANUAL)
            .replace('__LOCKSRC__', data['lock_src']).replace('__LOCKRATE__', str(data['lock_rate']))
            .replace('__PAIRNOTE__', pairnote).replace('__MINOUT__', str(int(MIN_OUTAGE_S)))
            .replace('__FILE__', data['file']).replace('__SPAN__', str(data['span']))
            .replace('__CRCBAD__', str(data['crc_bad'])).replace('__NCYC__', str(data['n_cycles']))
            .replace('__MAINMSG__', main_msg_name())
            .replace('__CYCTABLE__', cyctable)
            .replace('__AGG_OUT__', S(a['outage'])).replace('__AGG_S__', S(a['single']))
            .replace('__AGG_F__', S(a['float'])).replace('__AGG_X__', S(a['fixed']))
            .replace('__AGG_H__', S(a['fixed_hold'])).replace('__AGG_SF__', S(a['single_to_float']))
            .replace('__AGG_FF__', S(a['float_to_fixed']))
            .replace('__IMG1__', img64(imgs['keytime'])).replace('__IMG2__', img64(imgs['stages']))
            .replace('__IMG3__', img64(imgs['svs']))
            .replace('__LOCKCLAUSE__', lock_clause()).replace('__PTCLAUSE__', pt_clause())
            .replace('__SVCLAUSE__', sv_clause()).replace('__STNCLAUSE__', stn_clause()))
    # MD
    md = ['# %s' % TAG, '', '数据源：%s｜口径：%s' % (data['file'], MANUAL), '']
    md.append('## 一、周期切割')
    md.append('- 锁定源：%s（%.1fHz）；检出周期数：%d；时长 %ss' % (
        data['lock_src'], data['lock_rate'], data['n_cycles'], data['span']))
    md.append('')
    md.append('## 二、各周期关键时刻（自起点秒）')
    md.append('| 周期 | 断线时长 | 首单点/伪距类 | 首浮点 | 首固定 | 固定保持5s |')
    md.append('|---|---|---|---|---|---|')
    for c in data['cycles']:
        f = c['first']
        md.append('| #%d | %.0fs | %s | %s | %s | %s |' % (
            c['idx'], c['outage_dur'], V(f['single']), V(f['float']), V(f['fixed']), V(c['fixed_hold'])))
    md.append('')
    md.append('## 三、跨周期统计')
    for k, lab in [('outage', '断线时长'), ('single', '起点→单点'), ('float', '起点→浮点'),
                   ('fixed', '起点→固定'), ('fixed_hold', '起点→固定保持5s'),
                   ('single_to_float', '单点→浮点'), ('float_to_fixed', '浮点→固定')]:
        md.append('- %s：%s' % (lab, S(a[k])))
    return html, '\n'.join(md)


def main_msg_name():
    return 'BESTGNSSPOSA'


def lock_clause():
    return 'UG016 4.2.26 TRACKSTAT 字段12(locktime)/字段8(ch-tr-status)'


def pt_clause():
    return 'UG016 表4-2（SINGLE=16/NARROW_FLOAT=34/NARROW_INT=50）'


def sv_clause():
    return 'UG016 4.2.2 字段15/16/18'


def stn_clause():
    return 'UG016 4.2.2 字段12'


def run(paths, out_dir=None, open_browser=False):
    if not paths:
        raise SystemExit('未提供日志文件')
    if out_dir is None:
        out_dir = DEFAULT_OUT
    os.makedirs(out_dir, exist_ok=True)
    by_path = None
    hc_path = None
    for p in paths:
        low = os.path.basename(p).lower()
        if hc_path is None and ('m720' in low or 'huace' in low or 'com11' in low):
            hc_path = p
        elif by_path is None:
            by_path = p
    if by_path is None:
        by_path = paths[0]
        hc_path = paths[1] if len(paths) > 1 else None
    data = build(by_path, hc_path)
    imgs = make_images(data, out_dir)
    html, md = build_outputs(data, imgs)
    h_path = os.path.join(out_dir, 'report.html')
    m_path = os.path.join(out_dir, 'report.md')
    open(h_path, 'w', encoding='utf-8').write(html)
    open(m_path, 'w', encoding='utf-8').write(md)
    js_path = os.path.join(out_dir, 'summary.json')
    json.dump({k: v for k, v in data.items() if k != 'cycles'} | {'cycles': data['cycles'], 'agg': agg(data['cycles'])},
              open(js_path, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('生成 HTML:', h_path, '%.0f KB' % (len(html) / 1024))
    print('周期数  :', data['n_cycles'])
    if open_browser:
        webbrowser.open('file:///' + h_path.replace(chr(92), '/'))
    return h_path, m_path


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        print('用法: python BY_warmstart_.py <输出目录> <北云日志> [华测日志(可选,取早)]')
        raise SystemExit(2)
    out_dir = sys.argv[1]
    paths = [a for a in sys.argv[2:] if os.path.isfile(a)]
    run(paths, out_dir, open_browser=False)


if __name__ == '__main__':
    main()