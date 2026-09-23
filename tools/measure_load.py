#!/usr/bin/env python3
"""Log RDK CPU load for a while and print who uses it (BACKLOG #18/#23/#45).

    python3 tools/measure_load.py [--seconds 60] [--period 2] [--label race_idle] [--out DIR]

Every --period seconds it samples: 1/5/15 min load average, SoC and DDR temperature, CPU clock,
and every process's CPU % (from /proc, so no extra packages). At the end it prints the totals
and the top processes (mean and peak % of ONE core; 100 = one full core, the RDK X5 has 8) and
writes <out>/<label>_<time>.csv (one row per sample and process) for later comparison.
Run it with nothing else open on the robot; start it after the launch has settled (~60 s).
"""
import argparse
import csv
import os
import time
from collections import defaultdict

CLK = os.sysconf('SC_CLK_TCK')


def procs():
    """pid -> (name, cpu ticks). Name = the node (--ros-args __node:=) or the executable."""
    out = {}
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/stat') as f:
                stat = f.read()
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                cmd = f.read().replace(b'\0', b' ').decode(errors='replace').strip()
        except OSError:
            continue
        rest = stat[stat.rindex(')') + 2:].split()
        ticks = int(rest[11]) + int(rest[12])                    # utime + stime
        name = stat[stat.index('(') + 1:stat.rindex(')')]
        for tok in cmd.split():
            if tok.startswith('__node:='):
                name = tok[len('__node:='):]
                break
        else:
            parts = [t for t in cmd.split() if not t.startswith('-')]
            if parts:
                name = os.path.basename(parts[1] if name.startswith('python') and len(parts) > 1 else parts[0])
        out[int(pid)] = (name[:32], ticks)
    return out


def temp(kind):
    base = '/sys/class/thermal'
    try:
        for z in sorted(os.listdir(base)):
            if z.startswith('thermal_zone'):
                with open(f'{base}/{z}/type') as f:
                    if f.read().strip() == kind:
                        with open(f'{base}/{z}/temp') as g:
                            return int(g.read()) / 1000.0
    except OSError:
        pass
    return float('nan')


def freq():
    try:
        with open('/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq') as f:
            return int(f.read()) / 1000.0
    except OSError:
        return float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--period', type=float, default=2.0)
    ap.add_argument('--label', default='load')
    ap.add_argument('--out', default=os.path.expanduser('~/carbot_data/load'))
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    path = os.path.join(a.out, f'{a.label}_{time.strftime("%Y%m%d_%H%M%S")}.csv')

    per = defaultdict(list)            # name -> [% of one core per sample]
    load, cpu_t, ddr_t, mhz = [], [], [], []
    prev, t_prev = procs(), time.monotonic()
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(['t_s', 'load1', 'soc_c', 'ddr_c', 'cpu_mhz', 'process', 'cpu_pct'])
        t0 = t_prev
        while time.monotonic() - t0 < a.seconds:
            time.sleep(a.period)
            now = time.monotonic()
            cur = procs()
            dt = now - t_prev
            agg = defaultdict(float)
            for pid, (name, ticks) in cur.items():
                if pid in prev:
                    agg[name] += 100.0 * (ticks - prev[pid][1]) / CLK / dt
            for name, v in agg.items():
                per[name].append(v)
            l1 = os.getloadavg()[0]
            load.append(l1), cpu_t.append(temp('thermal-cpu')), ddr_t.append(temp('thermal-ddr')), mhz.append(freq())
            for name, v in agg.items():
                if v >= 1.0:
                    w.writerow([f'{now - t0:.1f}', f'{l1:.2f}', f'{cpu_t[-1]:.1f}', f'{ddr_t[-1]:.1f}',
                                f'{mhz[-1]:.0f}', name, f'{v:.1f}'])
            prev, t_prev = cur, now

    n = len(load)
    total = defaultdict(float)
    for i in range(n):
        for name, vals in per.items():
            if i < len(vals):
                total[i] += vals[i]
    print(f'== {a.label}: {n} samples over {a.seconds:.0f} s, {os.cpu_count()} cores')
    print(f'load1 last {load[-1]:.1f} max {max(load):.1f} | SoC {max(cpu_t):.1f} C max | DDR {max(ddr_t):.1f} C max'
          f' | CPU clock min {min(mhz):.0f} MHz')
    print(f'all processes together: mean {sum(total.values()) / n:.0f} % of one core '
          f'({sum(total.values()) / n / os.cpu_count():.0f} % of the machine)')
    print(f'{"process":34s} {"mean%":>6s} {"peak%":>6s}')
    for name, vals in sorted(per.items(), key=lambda kv: -sum(kv[1]))[:22]:
        print(f'{name:34s} {sum(vals) / n:6.1f} {max(vals):6.1f}')
    print(f'csv: {path}')


if __name__ == '__main__':
    main()
