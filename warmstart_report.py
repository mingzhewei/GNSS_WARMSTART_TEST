# -*- coding: utf-8 -*-
"""
GNSS 热启动 多设备汇总分析（统一时序对齐 + 统一取早 + 一份汇总报告）
=====================================================================
流程：
  1. 各设备用 gnss_parsers 中注册的 parser 解析，得到 locktime 周期 + 定位序列（绝对 GPS 周秒）。
  2. 周期对齐：放弃第 1 次循环（周秒未对齐）；第 2 次循环起用绝对周秒对齐，
     不同设备断线起点相差 <= PAIR_TOL_S 配为同一次循环。
  3. 统一 t0：同一循环取所有参与设备中最早的 locktime 翻正时刻。
  4. 各设备自统一 t0 起算单点/浮点/固定耗时与卫星恢复，出一份汇总 HTML/MD/PNG/JSON。

用法：python warmstart_report.py <输出目录> <日志1> [日志2 ...]
（每个日志的设备类型由 gnss_parsers.detect_parser 自动识别）
"""
import os
import sys
import io
import json
import base64
import statistics
import collections
import webbrowser

import gnss_parsers as G

ROOT = os.path.dirname(os.path.abspath(__file__))
DEFAULT_OUT = os.path.join(ROOT, 'reports')
TAG = 'GNSS热启动多设备汇总分析'

if sys.stdout.encoding and sys.stdout.encoding.lower() not in ('utf-8', 'utf8'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

CAT_CN = ['无解', '单点', 'RTK浮点', 'RTK固定']
WINDOW_AFTER = 180.0   # 起点后分析窗口（秒）
SKIP_FIRST = 1         # 放弃前 N 次循环（周秒未对齐）


def cat_of(parser, pt):
    return 3 if pt in parser.CAT3 else 2 if pt in parser.CAT2 else 1 if pt in parser.CAT1 else 0


def analyze_cycle(parser, t0, rows, after=WINDOW_AFTER):
    win = [r for r in rows if t0 - 0.05 <= r['t'] <= t0 + after]
    def first(pred):
        for r in win:
            if pred(r):
                return round(r['t'] - t0, 1)
        return None
    f = dict(single=first(lambda r: cat_of(parser, r['pt']) == 1),
             float=first(lambda r: cat_of(parser, r['pt']) == 2),
             fixed=first(lambda r: cat_of(parser, r['pt']) == 3),
             stn=first(lambda r: r['stn'] not in ('', '0')),
             svs10=first(lambda r: r['svs'] >= 10),
             svs20=first(lambda r: r['svs'] >= 20))
    fixed_hold = None
    for i, r in enumerate(win):
        if cat_of(parser, r['pt']) == 3:
            if all(cat_of(parser, x['pt']) == 3 for x in win[i:] if x['t'] <= r['t'] + 5.0):
                fixed_hold = round(r['t'] - t0, 1)
                break
    def dur(a, b):
        return None if (a is None or b is None) else round(b - a, 1)
    stages = dict(lock_to_single=f['single'], single_to_float=dur(f['single'], f['float']),
                  float_to_fixed=dur(f['float'], f['fixed']), lock_to_fixed=f['fixed'])
    series = dict(t=[round(r['t'] - t0, 1) for r in win], svs=[r['svs'] for r in win],
                  soln=[r['soln'] for r in win], multi=[r['multi'] for r in win],
                  cats=[cat_of(parser, r['pt']) for r in win])
    return dict(first=f, stages=stages, fixed_hold=fixed_hold, series=series, n=len(win))


def build_group(devices):
    """devices: [parser.parse(path), ...]。返回汇总结构。"""
    # 1) 各设备放弃前 SKIP_FIRST 次循环
    for d in devices:
        d['cycles_used'] = d['cycles'][SKIP_FIRST:]
    # 2) 以最周期数设备为基准，按断线起点跨设备配对（绝对周秒）
    ref = max(devices, key=lambda d: len(d['cycles_used']))
    n_cyc = len(ref['cycles_used'])
    groups = []
    for ci in range(n_cyc):
        ref_c = ref['cycles_used'][ci]
        members = []
        for d in devices:
            best, best_dt = None, None
            for c in d['cycles_used']:
                dt = abs(c['outage_start'] - ref_c['outage_start'])
                if dt <= G.PAIR_TOL_S and (best_dt is None or dt < best_dt):
                    best, best_dt = c, dt
            if best is not None:
                members.append((d, best))
        groups.append(members)
    # 3) 每组的统一 t0 = 所有成员最早 locktime；各成员分析
    out_cycles = []
    for gi, members in enumerate(groups):
        t0 = min(c['t_lock'] for _, c in members)
        per_dev = []
        for d, c in members:
            info = analyze_cycle(d['parser'], t0, d['pos_rows'])
            per_dev.append(dict(product=d['product'], display=d['display'], file=d['file'],
                                lock_src=d['lock_src'], lock_rate=d['lock_rate'],
                                t_lock=round(c['t_lock'] - t0, 2), outage_dur=c['outage_dur'],
                                **info))
        out_cycles.append(dict(idx=gi + SKIP_FIRST + 1, t0=t0, outage_dur=members[0][1]['outage_dur'],
                               n_dev=len(members), devs=per_dev))
    return dict(n_devices=len(devices), n_cycles=len(out_cycles), cycles=out_cycles,
                devices=[dict(product=d['product'], display=d['display'], file=d['file'],
                              lock_src=d['lock_src'], lock_rate=d['lock_rate'],
                              n_cycles=len(d['cycles_used']), crc_bad=d['crc_bad'],
                              span=d['span'], main_msg=d['parser'].main_msg(),
                              pos_integrity=d['pos_integrity'], lock_integrity=d['lock_integrity'],
                              lock_clause=d['parser'].lock_clause(), pt_clause=d['parser'].pt_clause(),
                              sv_clause=d['parser'].sv_clause(), stn_clause=d['parser'].stn_clause())
                         for d in devices])


def agg_dev(cycles, product):
    """某设备跨周期统计。"""
    def stats(getter):
        vals = []
        for c in cycles:
            for dv in c['devs']:
                if dv['product'] == product:
                    v = getter(dv)
                    if v is not None:
                        vals.append(v)
        if not vals:
            return None
        return dict(min=round(min(vals), 1), mean=round(statistics.mean(vals), 1),
                    max=round(max(vals), 1), n=len(vals))
    return dict(single=stats(lambda d: d['first']['single']),
                float=stats(lambda d: d['first']['float']),
                fixed=stats(lambda d: d['first']['fixed']),
                fixed_hold=stats(lambda d: d['fixed_hold']),
                single_to_float=stats(lambda d: d['stages']['single_to_float']),
                float_to_fixed=stats(lambda d: d['stages']['float_to_fixed']))
# ----------------------------------------------------------------------
# 绘图
# ----------------------------------------------------------------------

DEV_COLORS = ['#2563eb', '#dc2626', '#16a34a', '#9333ea', '#f59e0b', '#0891b2']


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
    products = []
    for c in cyc:
        for dv in c['devs']:
            if dv['product'] not in products:
                products.append(dv['product'])

    # 图1：各周期首固定耗时（分设备分组柱状）
    fig, ax = plt.subplots(figsize=(max(11, len(cyc) * 1.0), 4.8))
    w = 0.8 / max(1, len(products))
    for pi, prod in enumerate(products):
        ys = []
        for c in cyc:
            v = next((dv['first']['fixed'] for dv in c['devs'] if dv['product'] == prod), None)
            ys.append(v if v is not None else float('nan'))
        off = [x + (pi - (len(products) - 1) / 2) * w for x in xs]
        ax.bar(off, [0 if y != y else y for y in ys], width=w * 0.9, color=DEV_COLORS[pi % len(DEV_COLORS)],
               label=prod, zorder=3)
        for x, y in zip(off, ys):
            if y == y:
                ax.text(x, y, '%.1f' % y, ha='center', va='bottom', fontsize=7, color=DEV_COLORS[pi % len(DEV_COLORS)])
    ax.set_xticks(xs); ax.set_xticklabels(labs, fontsize=9)
    ax.set_xlabel('热启动周期'); ax.set_ylabel('自统一t0到首固定（秒）')
    ax.set_title('各设备 各热启动周期 首固定耗时（t0=多设备最早locktime）', fontsize=13)
    ax.grid(axis='y', ls='--', alpha=0.4); ax.legend(loc='upper left', fontsize=9)
    fig.tight_layout()
    paths['fixed'] = os.path.join(out_dir, 'img_首固定对比.png')
    fig.savefig(paths['fixed'], dpi=150); plt.close(fig)

    # 图2：各周期 单点/浮点/固定（分设备）
    fig, axes = plt.subplots(len(products), 1, figsize=(max(11, len(cyc) * 1.0), 3.2 * len(products)), squeeze=False)
    marks = [('single', '首单点', 's'), ('float', '首浮点', '^'), ('fixed', '首固定', 'D')]
    for pi, prod in enumerate(products):
        ax = axes[pi][0]
        for key, lab, mk in marks:
            ys = []
            for c in cyc:
                v = next((dv['first'][key] for dv in c['devs'] if dv['product'] == prod), None)
                ys.append(v if v is not None else float('nan'))
            ax.scatter(xs, ys, marker=mk, s=60, label=lab, zorder=3,
                       color={'single': '#3b82f6', 'float': '#f59e0b', 'fixed': '#16a34a'}[key])
        ax.set_xticks(xs); ax.set_xticklabels(labs, fontsize=8)
        ax.set_ylabel('耗时(s)'); ax.set_title('%s 关键时刻' % prod, fontsize=11)
        ax.grid(axis='y', ls='--', alpha=0.4); ax.legend(loc='upper left', ncols=3, fontsize=8)
    axes[-1][0].set_xlabel('热启动周期')
    fig.tight_layout()
    paths['keytime'] = os.path.join(out_dir, 'img_周期时刻.png')
    fig.savefig(paths['keytime'], dpi=150); plt.close(fig)

    # 图3：各周期 卫星数恢复曲线（每设备一列、每周期一行太多 -> 改为叠加曲线）
    nrow = len(cyc)
    fig, axes = plt.subplots(nrow, 1, figsize=(13, 2.2 * nrow), squeeze=False)
    for i, c in enumerate(cyc):
        ax = axes[i][0]
        for pi, dv in enumerate(c['devs']):
            s = dv['series']
            ax.plot(s['t'], s['svs'], lw=1.1, color=DEV_COLORS[pi % len(DEV_COLORS)],
                    label='%s #SVs' % dv['product'], drawstyle='steps-post')
        ax.set_title('周期 #%d（断线 %.0fs，t0=多设备最早locktime）' % (c['idx'], c['outage_dur']), fontsize=9)
        ax.set_xlim(0, 90)
        ax.grid(ls='--', alpha=0.3); ax.tick_params(labelsize=7.5)
        if i == 0:
            ax.legend(loc='lower right', fontsize=7.5)
    fig.suptitle('各周期 各设备 卫星数量恢复曲线', fontsize=12)
    fig.tight_layout(rect=[0, 0, 1, 0.99])
    paths['svs'] = os.path.join(out_dir, 'img_卫星数量时间轴.png')
    fig.savefig(paths['svs'], dpi=150); plt.close(fig)
    return paths


def img64(path):
    return 'data:image/png;base64,' + base64.b64encode(open(path, 'rb').read()).decode('ascii')


def V(v, u='s'):
    return '—' if v is None else ('%.1f%s' % (v, u))


def S(st):
    return '—' if st is None else '%.1f / %.1f / %.1f (n=%d)' % (st['min'], st['mean'], st['max'], st['n'])
# ----------------------------------------------------------------------
# HTML / MD 报告
# ----------------------------------------------------------------------

CSS = """
 *{box-sizing:border-box}
 body{font-family:"Microsoft YaHei","Segoe UI",sans-serif;margin:0;background:#f5f7fa;color:#1f2937}
 .wrap{max-width:1380px;margin:0 auto;padding:24px}
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
<h1>GNSS 热启动（断天线重接）多设备汇总收敛分析</h1>
<div class="sub">设备数：__NDEV__｜有效周期数：__NCYC__（已放弃第 1 次循环，自第 2 次起绝对GPS周秒对齐）｜统一 t0 = 多设备最早 locktime 翻正时刻｜周期切割：0 锁定 ≥ __MINOUT__s</div>

<h2>一、数据完整性检查（按报头时间戳逐帧自判，不假设频率）</h2>
<div class="card"><table>
<tr><th>设备</th><th>文件</th><th>报文</th><th>帧数</th><th>实测周期</th><th>实测采样频率</th><th>真实间断</th><th>时间轴跳变</th><th>CRC失败行</th></tr>
__DEVTABLE__
</table>
<div class="note">实测周期 = 相邻帧报头时间差的中位数（不假设）；实测采样频率 = 1/实测周期；真实间断 = 相邻帧时间差 &gt; 2.5×实测周期（保留在时间轴）；时间轴跳变 = 相邻帧时间差 &gt; 3600s，热启动场景下典型为：设备无电池备份RTC、断天线长时间无星后时标由 FREEWHEELING 失效退回默认周（UNKNOWN），重新捕获卫星后由导航电文重新解出真实周，此类跳变按时间连续处理。锁定源 = 各设备含 locktime 字段且实测频率最高的报文（动态选择）。起点不含"天线接上的物理瞬间"（需 ANTENNASTATUS/EVENT，本批未录制）。</div></div>

<h2>二、各热启动周期 各设备关键时刻（自统一t0秒）</h2>
__CYC_BLOCKS__
<div class="imgcard"><img src="__IMG_FIXED__"></div>
<div class="imgcard"><img src="__IMG_KEY__"></div>

<h2>三、跨周期统计（自统一t0秒）</h2>
__AGG_BLOCKS__

<h2>四、各周期 各设备 卫星数量恢复曲线</h2>
<div class="imgcard"><img src="__IMG_SVS__"></div>

<h2>附录：口径与手册条款对应</h2>
<div class="card"><table>
<tr><th>设备</th><th>起点(locktime)</th><th>定位类型</th><th>卫星计数</th><th>差分(基站ID)</th></tr>
__CLAUSE_TABLE__
</table></div>
</div></body></html>
"""


def build_outputs(data, imgs):
    products = [d['product'] for d in data['devices']]
    def _integ(d, msg, ig):
        return ("<tr><td>%s</td><td class='mono'>%s</td><td>%s</td><td>%d</td><td>%ss</td><td><b>%sHz</b></td>"
                "<td>%d</td><td>%d</td><td>%d</td></tr>" % (
                    d['display'], d['file'], msg, ig['frames'],
                    ig['period'], ig['rate'], len(ig['gaps']), len(ig['rebases']), d['crc_bad']))
    dev_rows = []
    for d in data['devices']:
        dev_rows.append(_integ(d, d['main_msg'], d['pos_integrity']))
        dev_rows.append(_integ(d, d['lock_src'] + '（锁定源）', d['lock_integrity']))
    devtable = ''.join(dev_rows)

    cyc_blocks = []
    for c in data['cycles']:
        rows = ''.join(
            "<tr><td>%s</td><td class='mono'>%s</td><td>%.1f</td><td>%.2f</td><td>%s</td><td>%s</td><td class='ok'>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
                dv['display'], dv['file'], dv['outage_dur'], dv['t_lock'],
                V(dv['first']['single']), V(dv['first']['float']), V(dv['first']['fixed']),
                V(dv['fixed_hold']), V(dv['first']['svs10']), V(dv['first']['svs20']))
            for dv in c['devs'])
        cyc_blocks.append(
            "<h3 style='font-size:14px;margin:14px 0 6px'>周期 #%d（断线 %.0fs，统一t0）</h3>"
            "<div class='card'><table><tr><th>设备</th><th>文件</th><th>断线时长</th><th>本机locktime(相对t0)</th>"
            "<th>首单点</th><th>首浮点</th><th>首固定</th><th>固定保持5s</th><th>星≥10</th><th>星≥20</th></tr>%s</table></div>"
            % (c['idx'], c['outage_dur'], rows))

    agg_blocks = []
    for prod in products:
        a = agg_dev(data['cycles'], prod)
        disp = next(d['display'] for d in data['devices'] if d['product'] == prod)
        agg_blocks.append(
            "<h3 style='font-size:14px;margin:14px 0 6px'>%s</h3><div class='card'><table>"
            "<tr><th>指标</th><th>min/均值/max(n)</th></tr>"
            "<tr><td>起点→首单点</td><td>%s</td></tr><tr><td>起点→首浮点</td><td>%s</td></tr>"
            "<tr><td>起点→首固定</td><td>%s</td></tr><tr><td>起点→固定保持5s</td><td>%s</td></tr>"
            "<tr><td>单点→浮点</td><td>%s</td></tr><tr><td>浮点→固定</td><td>%s</td></tr></table></div>" % (
                disp, S(a['single']), S(a['float']), S(a['fixed']), S(a['fixed_hold']),
                S(a['single_to_float']), S(a['float_to_fixed'])))

    clause_table = ''.join(
        "<tr><td>%s</td><td>%s</td><td>%s</td><td>%s</td><td>%s</td></tr>" % (
            d['display'], d['lock_clause'], d['pt_clause'], d['sv_clause'], d['stn_clause'])
        for d in data['devices'])

    html = (HTML.replace('__TAG__', TAG).replace('__CSS__', CSS)
            .replace('__NDEV__', str(data['n_devices'])).replace('__NCYC__', str(data['n_cycles']))
            .replace('__MINOUT__', str(int(G.MIN_OUTAGE_S)))
            .replace('__DEVTABLE__', devtable)
            .replace('__CYC_BLOCKS__', ''.join(cyc_blocks))
            .replace('__AGG_BLOCKS__', ''.join(agg_blocks))
            .replace('__CLAUSE_TABLE__', clause_table)
            .replace('__IMG_FIXED__', img64(imgs['fixed']))
            .replace('__IMG_KEY__', img64(imgs['keytime']))
            .replace('__IMG_SVS__', img64(imgs['svs'])))

    md = ['# %s' % TAG, '', '设备数 %d｜有效周期数 %d（放弃第1次循环）' % (data['n_devices'], data['n_cycles']), '']
    for prod in products:
        a = agg_dev(data['cycles'], prod)
        disp = next(d['display'] for d in data['devices'] if d['product'] == prod)
        md.append('## %s' % disp)
        for k, lab in [('single', '起点→单点'), ('float', '起点→浮点'), ('fixed', '起点→固定'), ('fixed_hold', '起点→固定保持5s')]:
            md.append('- %s：%s' % (lab, S(a[k])))
        md.append('')
    return html, '\n'.join(md)


def run(paths, out_dir=None, open_browser=False):
    if not paths:
        raise SystemExit('未提供日志文件')
    if out_dir is None:
        out_dir = DEFAULT_OUT
    os.makedirs(out_dir, exist_ok=True)
    devices = []
    for p in paths:
        parser = G.detect_parser(p)
        if parser is None:
            print('无法识别设备类型，跳过:', p)
            continue
        print('解析 %s -> %s' % (os.path.basename(p), parser.key))
        devices.append(parser.parse(p))
    if not devices:
        raise SystemExit('没有可识别的设备日志')
    data = build_group(devices)
    imgs = make_images(data, out_dir)
    html, md = build_outputs(data, imgs)
    h_path = os.path.join(out_dir, 'report.html')
    m_path = os.path.join(out_dir, 'report.md')
    open(h_path, 'w', encoding='utf-8').write(html)
    open(m_path, 'w', encoding='utf-8').write(md)
    js = os.path.join(out_dir, 'summary.json')
    json.dump({k: v for k, v in data.items() if k != 'cycles'} | {'cycles': data['cycles']},
              open(js, 'w', encoding='utf-8'), ensure_ascii=False, indent=1)
    print('生成 HTML:', h_path, '%.0f KB' % (len(html) / 1024))
    print('设备数  :', data['n_devices'], ' 有效周期数:', data['n_cycles'])
    if open_browser:
        webbrowser.open('file:///' + h_path.replace(chr(92), '/'))
    return h_path, m_path


def main():
    if len(sys.argv) < 3:
        print(__doc__)
        print('用法: python warmstart_report.py <输出目录> <日志1> [日志2 ...]')
        raise SystemExit(2)
    out_dir = sys.argv[1]
    paths = [a for a in sys.argv[2:] if os.path.isfile(a)]
    run(paths, out_dir, open_browser=False)


if __name__ == '__main__':
    main()