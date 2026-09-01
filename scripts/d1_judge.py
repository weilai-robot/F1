#!/usr/bin/env python3
"""
d1_judge.py — D1 算力维度判定自动化（采集→判定的最后一公里）

输入: 一个或多个场景采集目录 (按 doc/测试体系/D0_公共基础.md §2.2 采集, 如
      /data/f1_obs/S2_0901_1530), 目录名以 S1/S2/S3 开头自动识别场景。

  RUN/
    sys/psi.log vmstat.log mem.log thermal.log mpstat.log cyclictest.log   (resource_monitor.sh)
    nav_tmux.log            大脑终端留档 (FastLIO [ mapping ]/Open3D time_this_loc/Nav2 WARN)
    aimrt_journal.log       小脑日志 (备用)
    ctrl_period_*.csv       R5 控制周期统计 (test_logs/data_csv/ 拷入)
    ecat_stats_*.csv        R3/R4 总线统计 (test_logs/ecat_stats/ 拷入)
    topic_hz__Odometry.log  可选 (monitor 脚本 ROS_SETUP 模式产出)

输出: 终端 + <RUN>/d1_judge.md (多场景时写入第一个目录), 判定表按 D1 方案 §3。
退出码: 0=全部可用项 PASS  1=存在 FAIL  2=全部 NODATA

用法:
  python3 scripts/d1_judge.py /data/f1_obs/S1_* /data/f1_obs/S2_* /data/f1_obs/S3_*
"""
import argparse
import csv
import glob
import os
import re
import sys

ANSI = re.compile(r'\x1b\[[0-9;]*[A-Za-z]')
WARN_PAT = re.compile(
    r'missed its requested rate|took too long|exceeded [0-9.]+ ?ms? to compute|'
    r'controller frequency|missed updating', re.I)

PASS, FAIL, WARN, NODATA = 'PASS', 'FAIL', 'WARN', '—'


# ─────────────────────────── 解析器 ───────────────────────────

def read(path):
    try:
        with open(path, encoding='utf-8', errors='replace') as f:
            return f.read()
    except OSError:
        return ''


def first(path, pattern):
    fs = sorted(glob.glob(os.path.join(path, pattern)))
    return fs[0] if fs else None


def p_fastlio(run):
    """[ mapping ] ... ave total: X (s) → (max_ms, trend 后1/3均值/前1/3均值)"""
    txt = ANSI.sub('', read(os.path.join(run, 'nav_tmux.log')))
    vals = [float(m) * 1000 for m in re.findall(r'ave total:\s*([0-9.eE+-]+)', txt)]
    if not vals:
        return None
    k = max(3, len(vals) // 3)
    head, tail = vals[:k], vals[-k:]
    trend = (sum(tail) / len(tail)) / max(1e-9, sum(head) / len(head))
    return max(vals), trend, len(vals)


def p_open3d(run):
    txt = ANSI.sub('', read(os.path.join(run, 'nav_tmux.log')))
    vals = [float(m) * 1000 for m in re.findall(r'time_this_loc:\s*([0-9.eE+-]+)', txt)]
    return max(vals) if vals else None


def p_nav2warn(run):
    txt = ANSI.sub('', read(os.path.join(run, 'nav_tmux.log')))
    return sum(1 for _ in WARN_PAT.finditer(txt))


def p_odom_hz(run):
    txt = read(os.path.join(run, 'topic_hz__Odometry.log'))
    vals = [float(m) for m in re.findall(r'average rate:\s*([0-9.]+)', txt)]
    return vals[-1] if vals else None


def p_psi(run, kind):
    mx = -1.0
    for line in read(os.path.join(run, 'sys', 'psi.log')).splitlines():
        seg = re.search(kind + r':\s*(.*)', line)
        if not seg:
            continue
        m = re.search(r'some avg10=\S+ avg60=([0-9.]+)', seg.group(1)) if kind == 'cpu_some' \
            else re.search(r'some avg10=\S+ avg60=([0-9.]+)', seg.group(1))
        if m:
            mx = max(mx, float(m.group(1)))
    return mx if mx >= 0 else None


def p_vmstat(run):
    n = 0
    for line in read(os.path.join(run, 'sys', 'vmstat.log')).splitlines():
        f = line.split()
        if len(f) >= 8 and (f[6].isdigit() and int(f[6]) > 0 or f[7].isdigit() and int(f[7]) > 0):
            n += 1
    return n if os.path.exists(os.path.join(run, 'sys', 'vmstat.log')) else None


def p_mem_min(run):
    vals = [float(m) for m in re.findall(r'MemAvailable=([0-9.]+)', read(os.path.join(run, 'sys', 'mem.log')))]
    return min(vals) if vals else None


def p_thermal(run):
    temps = [int(t) for t in re.findall(r'max_temp=(\d+)', read(os.path.join(run, 'sys', 'thermal.log')))]
    thr = [int(t) for t in re.findall(r'total_throttle=(\d+)', read(os.path.join(run, 'sys', 'thermal.log')))]
    return (max(temps) / 1000.0, (thr[-1] - thr[0]) if thr else None) if temps else (None, None)


def p_iso(run, cpu):
    path = os.path.join(run, 'sys', 'mpstat.log')
    if not os.path.exists(path):
        return None
    for line in read(path).splitlines():
        f = line.split()
        if f[:2] == ['Average:', str(cpu)] and len(f) >= 6:
            return float(f[2]) + float(f[4])   # %usr + %sys
    return None


def p_busiest(run):
    best, path = None, os.path.join(run, 'sys', 'mpstat.log')
    if not os.path.exists(path):
        return None
    for line in read(path).splitlines():
        f = line.split()
        if len(f) >= 6 and f[1].isdigit() and f[1] not in ('4', '6'):
            v = float(f[2]) + float(f[4])
            if best is None or v > best[1]:
                best = (f'CPU{f[1]}', v)
    return best


def p_cyclictest(run):
    txt = read(os.path.join(run, 'sys', 'cyclictest.log'))
    m = re.findall(r'Max Latencies:\s*(\d+)', txt)
    return max(int(x) for x in m) if m else None


def p_ctrl_period(run):
    """R5 CSV → (max p99_late_us, miss 总数, 滑动10min窗 max miss, max late_max_us, rows)"""
    path = first(run, 'ctrl_period_*.csv')
    if not path:
        return None
    p99, miss_seq, lmax = [], [], 0
    with open(path, encoding='utf-8', errors='replace') as f:
        for r in csv.DictReader(f):
            p99.append(float(r['late_us_p99']))
            miss_seq.append(int(r['miss_cnt']))
            lmax = max(lmax, int(r['late_us_max']))
    if not p99:
        return None
    win = min(60, len(miss_seq))                      # 60 行 × 10s = 10min
    miss_win = max(sum(miss_seq[i:i + win]) for i in range(len(miss_seq) - win + 1))
    return max(p99), sum(miss_seq), miss_win, lmax, len(p99)


def p_ecat(run):
    """R3/R4 CSV → (wkc_bad 求和, dc_max_ns, 滑动10min窗 max overrun, rows)"""
    path = first(run, 'ecat_stats_*.csv')
    if not path:
        return None
    wkc, dcmax, over_seq = 0, 0, []
    with open(path, encoding='utf-8', errors='replace') as f:
        for r in csv.DictReader(f):
            wkc += int(r['wkc_bad_polls'])
            if r['dc_dev_abs_max_ns']:
                dcmax = max(dcmax, int(r['dc_dev_abs_max_ns']))
            over_seq.append(int(r['cycle_overruns']))
    if not over_seq:
        return None
    win = min(60, len(over_seq))
    over_win = max(sum(over_seq[i:i + win]) for i in range(len(over_seq) - win + 1))
    return wkc, dcmax, over_win, len(over_seq)


# ─────────────────────────── 判定表 ───────────────────────────
# (指标ID, 问题, 指标, 单位, 阈值说明, 提取器→(value, verdict))

def verdict(cmp_):
    """cmp_: None→NODATA; True→PASS; False→FAIL"""
    return NODATA if cmp_ is None else (PASS if cmp_ else FAIL)


def build_metrics(run, scen):
    fl = p_fastlio(run)
    dc_thr = 1.0 if scen == 'S1' else 5.0   # DC 偏差阈值 ns(µs×1000): S1<1µs, S2/S3<5µs
    cyc_thr = 50 if scen == 'S1' else 100
    M = []

    def add(mid, q, name, unit, thr_desc, val, verdict_):
        M.append((mid, q, name, unit, thr_desc, val, verdict_))

    # Q1
    if fl:
        trend_ok = fl[1] < 1.3 or fl[2] < 100
        v = PASS if (fl[0] < 70 and trend_ok) else (WARN if fl[0] < 70 else FAIL)
        add('fastlio', 'Q1', 'FastLIO ave total 最大/趋势', 'ms', '<70ms 且 后/前<1.3',
            f'{fl[0]:.1f} / ×{fl[1]:.2f} ({fl[2]}帧)', v)
    else:
        add('fastlio', 'Q1', 'FastLIO ave total 最大/趋势', 'ms', '<70ms 且 后/前<1.3', '—', NODATA)
    o3 = p_open3d(run)
    add('open3d', 'Q1', 'Open3D time_this_loc max', 'ms', '<100', f'{o3:.1f}' if o3 else '—',
        verdict(None if o3 is None else o3 < 100))
    w = p_nav2warn(run)
    nav_ok = None if not os.path.exists(os.path.join(run, 'nav_tmux.log')) else (w == 0)
    add('nav2warn', 'Q1', 'Nav2 超时/rate WARN 计数', '次', '=0', str(w) if nav_ok is not None else '—', verdict(nav_ok))
    hz = p_odom_hz(run)
    add('odomhz', 'Q1', '/Odometry 平均频率', 'Hz', '≥9.5', f'{hz:.2f}' if hz else '—', verdict(None if hz is None else hz >= 9.5))
    # Q2
    pc = p_psi(run, 'cpu_some')
    add('psicpu', 'Q2', 'PSI cpu some avg60 峰值', '%', '≤5', f'{pc:.1f}' if pc is not None else '—',
        verdict(None if pc is None else pc <= 5))
    pm = p_psi(run, 'mem')
    add('psimem', 'Q2', 'PSI mem some avg60 峰值', '%', '≤0.1', f'{pm:.2f}' if pm is not None else '—',
        verdict(None if pm is None else pm <= 0.1))
    sw = p_vmstat(run)
    add('swap', 'Q2', 'vmstat si/so>0 行数', '行', '=0', str(sw) if sw is not None else '—', verdict(None if sw is None else sw == 0))
    mm = p_mem_min(run)
    add('memmin', 'Q2', 'MemAvailable 最低', 'MB', '>2048', f'{mm:.0f}' if mm else '—', verdict(None if not mm else mm > 2048))
    tc, thd = p_thermal(run)
    add('temp', 'Q2', '峰值温度', '°C', '<90', f'{tc:.0f}' if tc else '—', verdict(None if not tc else tc < 90))
    add('throttle', 'Q2', 'throttle_count 增量', '次', '=0', str(thd) if thd is not None else '—',
        NODATA if thd is None else (PASS if thd == 0 else WARN))
    for cpu in (4, 6):
        u = p_iso(run, cpu)
        add(f'iso{cpu}', 'Q2', f'隔离核 CPU{cpu} %usr+%sys 平均', '%', '≤2', f'{u:.2f}' if u is not None else '—',
            verdict(None if u is None else u <= 2))
    b = p_busiest(run)
    add('busiest', 'Q2', '最忙非隔离核峰值', '%', '≤85(告警)', f'{b[0]} {b[1]:.1f}' if b else '—',
        NODATA if b is None else (PASS if b[1] <= 85 else WARN))
    # Q3
    cy = p_cyclictest(run)
    add('cyclic', 'Q3', 'cyclictest max 延迟', 'µs', f'<{cyc_thr}(场景相关)', str(cy) if cy else '—',
        verdict(None if not cy else cy < cyc_thr))
    cp = p_ctrl_period(run)
    if cp:
        p99_, miss, miss_win, lmax, rows = cp
        add('ctrlp99', 'Q3', 'Control 唤醒延迟 p99 (max)', 'µs', '<200', str(int(p99_)), verdict(p99_ < 200))
        add('ctrlmiss', 'Q3', 'deadline miss (总/滑窗10min)', '次', '滑窗<10',
            f'{miss}/{miss_win}', PASS if miss_win < 10 else FAIL)
    else:
        add('ctrlp99', 'Q3', 'Control 唤醒延迟 p99 (max)', 'µs', '<200', '—', NODATA)
        add('ctrlmiss', 'Q3', 'deadline miss (总/滑窗10min)', '次', '滑窗<10', '—', NODATA)
    ec = p_ecat(run)
    if ec:
        wkc, dcmax, over_win, rows = ec
        add('wkc', 'Q3', 'WKC 异常轮询合计', '次', '=0', str(wkc), verdict(wkc == 0))
        add('dcsync', 'Q3', 'DC 偏差 max', 'µs', f'<{dc_thr}(场景相关)', f'{dcmax/1000:.2f}',
            verdict(dcmax < dc_thr * 1000))
        add('ecover', 'Q3', '总线 overrun (滑窗10min)', '次', '<10', str(over_win), PASS if over_win < 10 else FAIL)
    else:
        for nm, nm2 in (('wkc', 'WKC 异常轮询合计'), ('dcsync', 'DC 偏差 max'), ('ecover', '总线 overrun')):
            add(nm, 'Q3', nm2, '—', '—', '—', NODATA)
    return M


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+', help='场景采集目录 (名以 S1/S2/S3 开头)')
    ap.add_argument('-o', '--out', default=None)
    a = ap.parse_args()

    scens = []
    for r in a.runs:
        m = re.match(r'(S[123])', os.path.basename(r.rstrip('/')))
        scens.append((m.group(1) if m else os.path.basename(r.rstrip('/')), r))
    table = {s: build_metrics(r, s) for s, r in scens}
    mids = [m[0] for m in next(iter(table.values()))]

    lines = ['# D1 算力维度判定表 (d1_judge.py)', '',
             '| 问题 | 指标 | 阈值 | ' + ' | '.join(s for s, _ in scens) + ' |',
             '|' + '---|' * (3 + len(scens))]
    verdicts = {s: {} for s, _ in scens}
    for mid in mids:
        rows = {}
        for s, _ in scens:
            row = next((m for m in table[s] if m[0] == mid), None)
            rows[s] = row
            verdicts[s][mid] = row[6] if row else NODATA
        base = next(r for r in rows.values() if r)
        q, name, unit, thr = base[1], base[2], base[3], base[4]
        cells = []
        for s, _ in scens:
            r = rows[s]
            cells.append(f'{r[5]} [{r[6]}]' if r else '—')
        lines.append(f'| {q} | {name} ({unit}) | {thr} | ' + ' | '.join(cells) + ' |')

    lines.append('')
    for s, _ in scens:
        for q in ('Q1', 'Q2', 'Q3'):
            vs = [v for mid, v in verdicts[s].items()
                  if next(m for m in table[s] if m[0] == mid)[1] == q]
            if all(v == NODATA for v in vs):
                g = NODATA
            elif any(v == FAIL for v in vs):
                g = FAIL
            elif all(v in (PASS, NODATA) for v in vs):
                g = PASS
            else:
                g = WARN
            lines.append(f'- **{s} {q}**: {g}')
    out = '\n'.join(lines) + '\n'
    print(out)
    dest = a.out or os.path.join(a.runs[0], 'd1_judge.md')
    try:
        with open(dest, 'w', encoding='utf-8') as f:
            f.write(out)
        print(f'已写入: {dest}', file=sys.stderr)
    except OSError as e:
        print(f'写入失败: {e}', file=sys.stderr)
    all_v = [v for s in verdicts.values() for v in s.values()]
    sys.exit(2 if all(v == NODATA for v in all_v) else (0 if all(v != FAIL for v in all_v) else 1))


if __name__ == '__main__':
    main()
