# -*- coding: utf-8 -*-
"""
GNSS 热启动 设备解析模块注册表（可插拔）
=====================================================================
设计目标：多设备同一轮"断天线->重接"热启动测试，统一时序对齐、统一取早起点、
统一出一份汇总报告。新增设备只需按 ParserBase 接口实现一个子类并注册到 PARSERS。

标准解析接口（ParserBase）：
  * key / display          设备唯一标识与显示名
  * match_path(path)       该文件是否属于本设备（用于 HMI 自动识别，可选）
  * parse(path) -> DeviceData，包含：
      - lock_times/lock_counts  locktime 锁定序列（绝对 GPS 周秒），用于切周期与定起点
      - pos_rows                定位主报文序列（绝对 GPS 周秒 + 定位类型 + 卫星数 + 基站ID）
      - lock_src/lock_rate      锁定源报文名与实测频率（动态选含 locktime 的最高频报文）
      - main_msg/counts/crc_bad/span  报告展示用

时序对齐口径（用户定义）：
  * 放弃第 1 次断线循环（接收机尚未持有卫星时间，周秒无法对齐）；
  * 第 2 次循环起各设备均已持有卫星时间，直接用绝对 GPS 周秒对齐；
  * 同一循环的统一 t0 = 所有参与设备中最早的 locktime 翻正时刻。

字段口径均依据各设备手册原文（见各 parser 的 docstring 与 clause()），非猜测。
"""
import os
import sys
import io
import collections
import struct

if sys.stdout.encoding and sys.stdout.encoding.lower() not in ('utf-8', 'utf8'):
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding='utf-8')

# 周期切割参数（全局统一）
MIN_OUTAGE_S = 8.0   # 0 锁定持续 >= 此值记为一次断线
MERGE_GAP_S = 8.0    # 相邻 0 锁定段间隔 < 此值合并（断线中的抖动）
PAIR_TOL_S = 30.0    # 不同设备断线起点相差 <= 此值才配为同一次循环


# ----------------------------------------------------------------------
# 通用工具
# ----------------------------------------------------------------------

def crc32b(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xEDB88320 if crc & 1 else crc >> 1
    return crc & 0xFFFFFFFF


def iter_msgs(path):
    """逐条产出 CRC 校验通过的报文 (报文名, 报头字段list, 数据字段list, kind)。

    '#' 开头按 32 位 CRC，'$' 开头按 NMEA 异或。二进制/残缺前缀不是本函数的
    输入对象；结尾产出 ('__BAD__', bad数, None, None)。
    """
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
                ok = (star + 9 <= len(line) and
                      crc32b(body) == int(line[star + 1:star + 9], 16))
            else:
                c = 0
                for bb in body:
                    c ^= bb
                ok = (star + 3 <= len(line) and
                      c == int(line[star + 1:star + 3], 16))
        except ValueError:
            ok = False
        if not ok:
            bad += 1
            continue
        txt = body.decode('ascii', 'ignore')
        if ';' in txt:
            h, b = txt.split(';', 1)
            hf = h.split(',')
            bf = b.split(',')
            yield hf[0], hf, bf, 'hash'
        else:
            yield txt.split(',')[0], None, txt.split(','), 'nmea'
    yield '__BAD__', bad, None, None


def pick_highest_freq(cands):
    """从 {报文名: [(t, locked), ...]} 中选实测频率最高者。
    返回 (报文名, times, locked, period)。这是"动态选高频锁定源"的判断函数。"""
    best = None
    for name, seq in cands.items():
        if len(seq) < 3:
            continue
        dts = sorted(seq[k][0] - seq[k - 1][0] for k in range(1, len(seq)))
        period = dts[len(dts) // 2]
        if best is None or period < best[3]:
            best = (name, [x[0] for x in seq], [x[1] for x in seq], period)
    if best is None:
        raise SystemExit('未找到含 locktime 的报文')
    return best


def cut_cycles(times, locked, min_outage=MIN_OUTAGE_S, merge_gap=MERGE_GAP_S):
    """0 锁定段 -> 周期列表 [{outage_start, outage_end, t_lock, outage_dur, lock_idx}]（绝对周秒）。"""
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
        cycles.append(dict(outage_start=times[a], outage_end=times[b], t_lock=times[nxt],
                           outage_dur=round(dur, 2), lock_idx=nxt))
    # 事件窗口必须在下一周期断线开始前截止，避免把后续周期状态混入当前周期。
    for i, c in enumerate(cycles):
        c['next_outage_start'] = cycles[i + 1]['outage_start'] if i + 1 < len(cycles) else None
    return cycles


def cat_of(pt, cat1, cat2, cat3):
    return 3 if pt in cat3 else 2 if pt in cat2 else 1 if pt in cat1 else 0


class ParserBase:
    key = 'base'
    display = '基类'
    CAT1 = set()
    CAT2 = set()
    CAT3 = set()

    def match_path(self, path):
        return False

    def parse(self, path):
        raise NotImplementedError

    # 报告展示用的手册条款（各设备覆盖）
    def lock_clause(self):
        return ''
    def pt_clause(self):
        return ''
    def sv_clause(self):
        return ''
    def stn_clause(self):
        return ''
    def main_msg(self):
        return ''


PARSERS = {}


def register(cls):
    PARSERS[cls.key] = cls()
    return cls





def iter_unicore_binary(path, msg_id=12):
    """按 N4 表7-49/7-50 解析二进制帧，并校验表7-48 的 32 位 CRC。

    本函数用于含 Unicore 二进制流的日志。帧头 AA44B5。帧长为
    24 字节 Header + MessageLength + 4 字节 CRC（小端）。
    """
    raw = open(path, 'rb').read()
    start = 0
    while True:
        pos = raw.find(b'\xaa\x44\xb5', start)
        if pos < 0:
            break
        start = pos + 1
        if pos + 28 > len(raw):
            continue
        length = int.from_bytes(raw[pos + 6:pos + 8], 'little')
        msg = int.from_bytes(raw[pos + 4:pos + 6], 'little')
        end = pos + 24 + length
        if msg != msg_id or end + 4 > len(raw) or length < 4:
            continue
        if crc32b(raw[pos:end]) != int.from_bytes(raw[end:end + 4], 'little'):
            continue
        week = int.from_bytes(raw[pos + 10:pos + 12], 'little')
        ms = int.from_bytes(raw[pos + 12:pos + 16], 'little')
        yield week * 604800.0 + ms / 1000.0, raw[pos + 24:end]

def detect_parser(path):
    """按内容自动识别设备（用于 HMI 提示）。识别不了返回 None。"""
    for p in PARSERS.values():
        try:
            if p.match_path(path):
                return p
        except Exception:
            continue
    return None

# ======================================================================
# 北云（UG016）
# ======================================================================

@register
class BeiyunParser(ParserBase):
    """北云 UG016。锁定源 TRACKSTATA(4.2.26 字段12 locktime)；定位 BESTGNSSPOSA(4.2.2)。"""
    key = 'beiyun'
    display = '北云 BY（UG016，BESTGNSSPOSA+TRACKSTATA）'
    CAT1 = {'SINGLE', 'PSRDIFF', 'WAAS', 'INS_PSRSP', 'INS_PSRDIFF', 'INS_SBAS'}
    CAT2 = {'L1_FLOAT', 'IONOFREE_FLOAT', 'NARROW_FLOAT', 'FLOATCONV', 'WIDELANE',
            'NARROWLANE', 'INS_RTKFLOAT'}
    CAT3 = {'L1_INT', 'WIDE_INT', 'NARROW_INT', 'INS_RTKFIXED', 'RTK_DIRECT_INS', 'PPP', 'INS_PPP'}

    def _htime(self, hf):
        try:
            return int(hf[5]) * 604800 + float(hf[6])
        except Exception:
            return None

    def match_path(self, path):
        n = 0
        for name, hf, bf, kind in iter_msgs(path):
            if name == '__BAD__':
                break
            if name.startswith('TRACKSTAT') or name.startswith('INSPVAXA'):
                return True
            n += 1
            if n > 3000:
                break
        return False

    def _trackstat_locked(self, bf):
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

    def parse(self, path):
        lock_cands = collections.defaultdict(list)
        pos_rows = []
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
            if hf is None:
                continue
            t = self._htime(hf)
            if t is None:
                continue
            if name.startswith('TRACKSTAT'):
                lock_cands['TRACKSTATA'].append((t, self._trackstat_locked(bf)))
            elif name.startswith('BESTGNSSPOS'):
                if len(bf) < 21:
                    continue
                try:
                    pos_rows.append(dict(t=t, ts=hf[4], sol=bf[0], pt=bf[1],
                                         stn=bf[10].strip('"'), age=float(bf[11]),
                                         svs=int(bf[13]), soln=int(bf[14]), multi=int(bf[16])))
                except Exception:
                    continue
        src, times, locked, period = pick_highest_freq(lock_cands)
        cycles = cut_cycles(times, locked)
        pos_integrity = frame_integrity([r['t'] for r in pos_rows])
        lock_integrity = frame_integrity(times)
        return dict(product=self.key, display=self.display, file=os.path.basename(path),
                    lock_times=times, lock_counts=locked, lock_src=src,
                    lock_rate=round(1.0 / period, 1), cycles=cycles, pos_rows=pos_rows,
                    pos_src='BESTGNSSPOSA',
                    counts=dict(counts), other=dict(other), crc_bad=bad,
                    pos_integrity=pos_integrity, lock_integrity=lock_integrity,
                    span=round(pos_rows[-1]['t'] - pos_rows[0]['t'], 1) if pos_rows else 0,
                    parser=self)

    def lock_clause(self):
        return 'UG016 4.2.26 TRACKSTAT 字段12(locktime)/字段8(ch-tr-status)'
    def pt_clause(self):
        return 'UG016 表4-2（SINGLE=16/NARROW_FLOAT=34/NARROW_INT=50）'
    def sv_clause(self):
        return 'UG016 4.2.2 字段15/16/18'
    def stn_clause(self):
        return 'UG016 4.2.2 字段12'
    def main_msg(self):
        return 'BESTGNSSPOSA'


# ======================================================================
# 华测（M720）
# ======================================================================

@register
class HuaceParser(ParserBase):
    """华测 M720。锁定源 RANGEA(3.2.25 字段14 locktime，1Hz)；定位主报文优先使用连续 BESTPA。
    双报文格式：GMF(week=字段6,TOW=字段7 ms) / NovAtel(week=字段5,sec=字段6 s)。"""
    key = 'huace'
    display = '华测 HUACE（M720，BESTPA+RANGEA）'
    CAT1 = {'SINGLE', 'PSRDIFF', 'SPPDIFF', 'WAAS', 'SBAS', 'INS_PSRSP', 'INS_PSRDIFF', 'INS_SBAS', 'INS_SPP'}
    CAT2 = {'L1_FLOAT', 'IONOFREE_FLOAT', 'NARROW_FLOAT', 'FLOATCONV', 'WIDELANE', 'NARROWLANE',
            'INS_RTKFLOAT', 'PPP_CONVERGING'}
    CAT3 = {'L1_INT', 'WIDE_INT', 'NARROW_INT', 'INS_RTKFIXED', 'RTK_DIRECT_INS', 'PPP', 'PPP_FIXED', 'INS_PPP'}
    GMF_MSGS = {'RTKPA', 'RTKVA', 'BESTPA', 'BESTVA', 'BESTDOPSA', 'ENVSTATUSA'}

    def _htime(self, hf, name):
        try:
            if name in self.GMF_MSGS:
                return int(hf[6]) * 604800 + float(hf[7]) / 1000.0
            return int(hf[5]) * 604800 + float(hf[6])
        except Exception:
            return None

    def match_path(self, path):
        n = 0
        for name, hf, bf, kind in iter_msgs(path):
            if name == '__BAD__':
                break
            if name in ('RTKPA', 'ENVSTATUSA', 'SATVIS2A', 'RANGEA'):
                return True
            n += 1
            if n > 3000:
                break
        return False

    def _range_locked(self, bf):
        try:
            nobs = int(bf[0])
        except Exception:
            return 0
        recs = bf[1:]
        per = len(recs) // nobs if nobs else 0
        locked = 0
        for i in range(nobs):
            rec = recs[i * per:(i + 1) * per]
            if len(rec) < per:
                continue
            try:
                cno = float(rec[7]); lt = float(rec[8])
            except Exception:
                continue
            if lt > 0 and cno > 0:
                locked += 1
        return locked

    def _gnssobs_locked(self, bf):
        try:
            nobs = int(bf[0])
        except Exception:
            return 0
        recs = bf[1:]
        per = len(recs) // nobs if nobs else 0
        locked = 0
        for i in range(nobs):
            rec = recs[i * per:(i + 1) * per]
            if len(rec) < per:
                continue
            try:
                cno = float(rec[7]); lt = float(rec[13])
            except Exception:
                continue
            if lt > 0 and cno > 0:
                locked += 1
        return locked

    @staticmethod
    def _pos_row(t, hf, bf):
        """GMF BESTPA/RTKPA 的公共 body 字段映射（0 基）。"""
        if len(bf) < 23:
            raise ValueError('short position body')
        return dict(t=t, ts=hf[5], sol=bf[0], pt=bf[1],
                    stn=bf[22].strip('"'), age=float(bf[11]),
                    svs=int(bf[13]), soln=int(bf[14]), multi=int(bf[15]))

    def parse(self, path):
        lock_cands = collections.defaultdict(list)
        pos_candidates = collections.defaultdict(list)
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
            if hf is None:
                continue
            base = name.rstrip('A')
            if base in ('RANGE', 'RANGECMP'):
                t = self._htime(hf, name)
                if t is not None:
                    lock_cands[name].append((t, self._range_locked(bf)))
            elif base == 'GNSSOBS':
                t = self._htime(hf, name)
                if t is not None:
                    lock_cands[name].append((t, self._gnssobs_locked(bf)))
            elif name in ('BESTPA', 'RTKPA'):
                t = self._htime(hf, name)
                if t is None:
                    continue
                try:
                    pos_candidates[name].append(self._pos_row(t, hf, bf))
                except (ValueError, IndexError):
                    continue
        # BESTPA 是本批数据中的连续定位状态流；RTKPA 存在长间断，不能可靠判断首次事件。
        # 旧数据未录制 BESTPA 时保留 RTKPA 兜底，完整性表会暴露其采样问题。
        if pos_candidates.get('BESTPA'):
            pos_src, pos_rows = 'BESTPA', pos_candidates['BESTPA']
        elif pos_candidates.get('RTKPA'):
            pos_src, pos_rows = 'RTKPA', pos_candidates['RTKPA']
        else:
            pos_src, pos_rows = '', []
        src, times, locked, period = pick_highest_freq(lock_cands)
        cycles = cut_cycles(times, locked)
        pos_integrity = frame_integrity([r['t'] for r in pos_rows])
        lock_integrity = frame_integrity(times)
        return dict(product=self.key, display=self.display, file=os.path.basename(path),
                    lock_times=times, lock_counts=locked, lock_src=src,
                    lock_rate=round(1.0 / period, 1), cycles=cycles, pos_rows=pos_rows,
                    pos_src=pos_src,
                    counts=dict(counts), other=dict(other), crc_bad=bad,
                    pos_integrity=pos_integrity, lock_integrity=lock_integrity,
                    span=round(pos_rows[-1]['t'] - pos_rows[0]['t'], 1) if pos_rows else 0,
                    parser=self)

    def lock_clause(self):
        return ('RANGEA 本文件观测记录含 locktime；其“连续跟踪、无周跳”语义按 M7 V2.7 '
                'GNSSOBS/BASEOBS locktime 字段核对')
    def pt_clause(self):
        return 'M7 V2.7 3.3.3 BESTP_1 表3-107；Position Type 表3-109'
    def sv_clause(self):
        return 'M7 V2.7 3.3.3 BESTP_1 表3-107 字段15/16/17'
    def stn_clause(self):
        return 'M7 V2.7 3.3.3 BESTP_1 表3-107 字段24'
    def main_msg(self):
        return 'BESTPA（定位状态）+ RANGEA（locktime）'

# ----------------------------------------------------------------------
# 逐帧完整性检查（按报头时间戳自判，不假设频率）
# ----------------------------------------------------------------------

# ======================================================================
# 北斗（Unicore UM980）
# ======================================================================

@register
class BeidouParser(ParserBase):
    """北斗 UM980（Unicore N4）。定位 BESTNAVA；锁定源 OBSVMB(Message ID=12)。

OBSVMB 与 ASCII 报文混录在同一日志，必须按 AA44B5 二进制帧扫描并校验
N4 32 位 CRC，再按 OBSVM 表7-137 的 40 字节记录解码。
    """
    key = 'beidou'
    display = '北斗 BD（UM980，BESTNAVA+OBSVMB）'
    # N4 表0-5：伪距类、RTK浮点、RTK固定；PPP 不作为固定解归类。
    CAT1 = {'SINGLE', 'PSRDIFF', 'SBAS'}
    CAT2 = {'L1_FLOAT', 'IONOFREE_FLOAT', 'NARROW_FLOAT', 'PPP_CONVERGING'}
    CAT3 = {'L1_INT', 'WIDE_INT', 'NARROW_INT'}

    @staticmethod
    def _htime(hf):
        try:
            # N4 ASCII Header 表7-51：字段5=GPS周数，字段6=周内毫秒。
            return int(hf[4]) * 604800.0 + int(hf[5]) / 1000.0
        except Exception:
            return None

    def match_path(self, path):
        n = 0
        for name, hf, bf, kind in iter_msgs(path):
            if name == '__BAD__':
                continue
            if name in ('BESTNAVA', 'SATSINFOA', 'BESTSATA', 'RTKSTATUSA'):
                return True
            n += 1
            if n > 3000:
                break
        # iter_msgs 只覆盖 ASCII 行；关键锁定源是混录的 OBSVMB 二进制帧。
        try:
            for t, payload in iter_unicore_binary(path, msg_id=12):
                return True
        except OSError:
            pass
        return False

    @staticmethod
    def _obsvm_locked(payload):
        """按 N4 OBSVM 表7-137 解码所有 40 字节频点观测记录。"""
        try:
            nobs = int.from_bytes(payload[0:4], 'little')
        except Exception:
            return 0
        if len(payload) < 4 + nobs * 40:
            return 0
        locked = 0
        for i in range(nobs):
            off = 4 + i * 40
            rec = payload[off:off + 40]
            try:
                # 字段10 C/N0 在记录偏移28（H+32），值*100，
                # 字段12 locktime 在记录偏移32（H+36），单位s。
                cno = int.from_bytes(rec[28:30], 'little') / 100.0
                locktime = struct.unpack('<f', rec[32:36])[0]
            except Exception:
                continue
            if locktime > 0.0 and cno > 0.0:
                locked += 1
        return locked

    @staticmethod
    def _pos_row(t, hf, bf):
        if len(bf) < 15:
            raise ValueError('short BESTNAV body')
        return dict(t=t, ts=hf[3], sol=bf[0], pt=bf[1],
                    stn=bf[10].strip('"'), age=float(bf[11]),
                    svs=int(bf[13]), soln=int(bf[14]), multi=0)

    def parse(self, path):
        counts = collections.Counter()
        other = collections.Counter()
        bad = 0
        pos_rows = []
        for name, hf, bf, kind in iter_msgs(path):
            if name == '__BAD__':
                bad = hf if hf is not None else 0
                continue
            if kind == 'hash':
                counts[name] += 1
            else:
                other[name] += 1
            if name != 'BESTNAVA' or hf is None:
                continue
            t = self._htime(hf)
            if t is None:
                continue
            try:
                pos_rows.append(self._pos_row(t, hf, bf))
            except (ValueError, IndexError):
                continue

        times, locked = [], []
        binary_bad = 0
        raw = open(path, 'rb').read()
        start = 0
        while True:
            pos = raw.find(b'\xaa\x44\xb5', start)
            if pos < 0:
                break
            start = pos + 1
            if pos + 28 > len(raw):
                continue
            length = int.from_bytes(raw[pos + 6:pos + 8], 'little')
            msg = int.from_bytes(raw[pos + 4:pos + 6], 'little')
            end = pos + 24 + length
            if msg != 12 or end + 4 > len(raw) or length < 4:
                continue
            stored = int.from_bytes(raw[end:end + 4], 'little')
            if crc32b(raw[pos:end]) != stored:
                binary_bad += 1
                continue
            week = int.from_bytes(raw[pos + 10:pos + 12], 'little')
            ms = int.from_bytes(raw[pos + 12:pos + 16], 'little')
            times.append(week * 604800.0 + ms / 1000.0)
            locked.append(self._obsvm_locked(raw[pos + 24:end]))
        if len(times) < 3:
            raise SystemExit('未找到 CRC 校验通过的 OBSVMB 二进制观测帧')
        cycles = cut_cycles(times, locked)
        pos_integrity = frame_integrity([r['t'] for r in pos_rows])
        lock_integrity = frame_integrity(times)
        return dict(product=self.key, display=self.display, file=os.path.basename(path),
                    lock_times=times, lock_counts=locked, lock_src='OBSVMB',
                    lock_rate=round(1.0 / lock_integrity['period'], 1), cycles=cycles,
                    pos_rows=pos_rows, pos_src='BESTNAVA',
                    counts=dict(counts), other=dict(other), crc_bad=bad + binary_bad,
                    pos_integrity=pos_integrity, lock_integrity=lock_integrity,
                    span=round(pos_rows[-1]['t'] - pos_rows[0]['t'], 1) if pos_rows else 0,
                    parser=self)

    def lock_clause(self):
        return 'N4 R1.15 7.3.65 OBSVM 表7-137 字段10(C/N0)/字段12(locktime)'
    def pt_clause(self):
        return 'N4 R1.15 表0-5（SINGLE=16/PSRDIFF=17/NARROW_FLOAT=34/NARROW_INT=50）'
    def sv_clause(self):
        return 'N4 R1.15 7.3.19 BESTNAV 表7-70 字段15(#SVs)/16(#solnSVs)'
    def stn_clause(self):
        return 'N4 R1.15 7.3.19 BESTNAV 表7-70 字段12(stn id)'
    def main_msg(self):
        return 'BESTNAVA（定位状态）+ OBSVMB（locktime）'



def frame_integrity(times):
    """由报头时间序列计算完整性指标：
    实测周期 = 相邻帧时间差中位数；真实间断 = 2.5*周期 < dt <= 3600s；
    时间轴跳变(rebases) = |dt|>3600s，热启动场景下典型为：设备无电池备份RTC，
    断天线长时间无星后时标由 FREEWHEELING 失效退回默认周(UNKNOWN)，
    重新捕获卫星后由导航电文重新解出真实周。该类跳变按时间连续处理，
    间隔按实测周期计。返回 dict(frames, period, rate, gaps, rebases, max_dt)。"""
    n = len(times)
    if n < 2:
        return dict(frames=n, period=None, rate=None, gaps=[], rebases=[], max_dt=0.0)
    dts = [times[k] - times[k - 1] for k in range(1, n)]
    norm = [d for d in dts if abs(d) <= 3600.0]
    base = norm if norm else [abs(d) for d in dts]
    sd = sorted(base)
    period = sd[len(sd) // 2] if sd else 0.0
    gaps = []
    rebases = []
    el = 0.0
    for k, d in enumerate(dts):
        if abs(d) > 3600.0:
            rebases.append(dict(idx=k + 1, dt=round(d, 1)))
        elif period and d > period * 2.5:
            gaps.append(dict(idx=k + 1, dt=round(d, 2)))
    rate = (1.0 / period) if period else None
    return dict(frames=n, period=round(period, 4) if period else None,
                rate=round(rate, 2) if rate else None, gaps=gaps, rebases=rebases,
                max_dt=round(max((abs(d) for d in dts), default=0.0), 2))
