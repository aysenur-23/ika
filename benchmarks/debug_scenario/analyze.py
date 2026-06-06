#!/usr/bin/env python3
"""Faz 0/1 — baseline CSV özetleyici.

Kullanım:
  python3 analyze.py results/baseline_n10.csv
  python3 analyze.py results/baseline_n10.csv --markdown >> ../../docs/failure_taxonomy.md

Çıktı:
  - PASS rate
  - Başarısızlık modu dağılımı (FAIL_COLL, FAIL_TIMEOUT, FAIL_NAV, FAIL_LAUNCH)
  - Bitiş pozisyonu istatistikleri (ortalama, std)
  - Min engel mesafesi istatistikleri
  - Süre istatistikleri
  - (--markdown ise) Markdown tablo
"""
import argparse
import csv
import math
import statistics
import sys
from collections import Counter
from pathlib import Path


def _bool(r, key):
    """CSV'den bool oku — '1'/'true'/'True' kabul; eksik → None."""
    if key not in r or r[key] in ('', None):
        return None
    v = str(r[key]).strip().lower()
    if v in ('1', 'true', 'yes'):
        return True
    if v in ('0', 'false', 'no'):
        return False
    return None


def summarize(rows):
    total = len(rows)
    status_counts = Counter(r['status'] for r in rows)
    passes = status_counts.get('PASS', 0)

    # TASK-2: strict PASS — yeni alan varsa kullan, yoksa hesaplanabilir mi?
    strict_vals = [_bool(r, 'pass_strict') for r in rows]
    strict_passes = sum(1 for v in strict_vals if v is True)
    strict_available = any(v is not None for v in strict_vals)

    def num(rs, key, valid_only=True):
        vals = []
        for r in rs:
            try:
                v = float(r[key])
                if valid_only and (math.isnan(v) or v < -100):
                    continue
                vals.append(v)
            except (ValueError, KeyError, TypeError):
                continue
        return vals

    dist = num(rows, 'dist_to_goal')
    min_obs = num(rows, 'min_obs_dist')
    dur = num(rows, 'duration')
    # TASK-2 yeni metrikler — header yoksa hepsi boş döner
    min_obs_strict = num(rows, 'min_obstacle_distance')
    max_dev = num(rows, 'max_y_deviation')
    final_yerr = num(rows, 'final_y_error')
    transitions = num(rows, 'state_transition_count')
    stuck = num(rows, 'stuck_time')
    osc = num(rows, 'cmd_vel_oscillation_score')
    trial_dur = num(rows, 'trial_duration')
    # TASK-3.1: gerçek trial başlangıç pozu
    tsx = num(rows, 'trial_start_x')
    tsy = num(rows, 'trial_start_y')

    def stats(vals):
        if not vals:
            return None
        return {
            'n': len(vals),
            'mean': statistics.fmean(vals),
            'std': statistics.pstdev(vals) if len(vals) > 1 else 0.0,
            'min': min(vals),
            'max': max(vals),
        }

    return {
        'total': total,
        'passes': passes,
        'pass_rate': (100.0 * passes / total) if total else 0.0,
        'status_counts': status_counts,
        'dist_to_goal': stats(dist),
        'min_obs_dist': stats(min_obs),
        'duration': stats(dur),
        # TASK-2
        'strict_available': strict_available,
        'strict_passes': strict_passes,
        'strict_pass_rate': (100.0 * strict_passes / total) if total else 0.0,
        'min_obstacle_distance': stats(min_obs_strict),
        'max_y_deviation': stats(max_dev),
        'final_y_error_abs': stats([abs(v) for v in final_yerr]),
        'state_transition_count': stats(transitions),
        'stuck_time': stats(stuck),
        'cmd_vel_oscillation_score': stats(osc),
        'trial_duration': stats(trial_dur),
        # TASK-3.1
        'trial_start_x': stats(tsx),
        'trial_start_y': stats(tsy),
        'trial_start_available': bool(tsx) or bool(tsy),
    }


def fmt_stats(s):
    if s is None:
        return '—'
    return f"μ={s['mean']:.2f} σ={s['std']:.2f} [{s['min']:.2f},{s['max']:.2f}] n={s['n']}"


def print_text(summary, csv_path):
    print(f"=== {csv_path} ===")
    print(f"Total trials: {summary['total']}")
    print(f"PASS rate   : {summary['passes']}/{summary['total']} "
          f"({summary['pass_rate']:.1f}%)")
    print(f"")
    print(f"Status breakdown:")
    for status, count in sorted(summary['status_counts'].items(),
                                key=lambda kv: -kv[1]):
        pct = 100.0 * count / summary['total']
        print(f"  {status:15s} {count:3d} ({pct:5.1f}%)")
    print(f"")
    print(f"dist_to_goal (m)  : {fmt_stats(summary['dist_to_goal'])}")
    print(f"min_obs_dist (m)  : {fmt_stats(summary['min_obs_dist'])}")
    print(f"duration (s)      : {fmt_stats(summary['duration'])}")
    # TASK-2
    if summary['strict_available']:
        print("")
        print(f"STRICT PASS rate  : {summary['strict_passes']}/{summary['total']} "
              f"({summary['strict_pass_rate']:.1f}%)")
        print(f"min_obstacle_dist : {fmt_stats(summary['min_obstacle_distance'])}")
        print(f"max_y_deviation   : {fmt_stats(summary['max_y_deviation'])}")
        print(f"|final_y_error|   : {fmt_stats(summary['final_y_error_abs'])}")
        print(f"state_transitions : {fmt_stats(summary['state_transition_count'])}")
        print(f"stuck_time (s)    : {fmt_stats(summary['stuck_time'])}")
        print(f"osc_score [0..1]  : {fmt_stats(summary['cmd_vel_oscillation_score'])}")
        print(f"trial_duration (s): {fmt_stats(summary['trial_duration'])}")
    if summary['trial_start_available']:
        print("")
        print(f"trial_start_x (m) : {fmt_stats(summary['trial_start_x'])}")
        print(f"trial_start_y (m) : {fmt_stats(summary['trial_start_y'])}")
        s = summary['trial_start_x']
        if s and abs(s['mean']) > 0.5:
            print(f"  ⚠️  WARNING: trial_start_x ortalaması {s['mean']:.2f}m "
                  f"(> 0.5m). Robot trial başlamadan hareket etmiş olabilir; "
                  f"auto_start gating'i kontrol edin.")


def print_markdown(summary, csv_path):
    print(f"### Baseline: `{Path(csv_path).name}`")
    print()
    print(f"- **Toplam koşum:** {summary['total']}")
    print(f"- **PASS oranı:** {summary['passes']}/{summary['total']} "
          f"(**{summary['pass_rate']:.1f}%**)")
    print()
    print("| Sonuç | Sayı | Yüzde |")
    print("|---|---:|---:|")
    for status, count in sorted(summary['status_counts'].items(),
                                key=lambda kv: -kv[1]):
        pct = 100.0 * count / summary['total']
        print(f"| `{status}` | {count} | {pct:.1f}% |")
    print()
    print("| Metrik | μ | σ | min | max |")
    print("|---|---:|---:|---:|---:|")
    metrics = [('Hedefe uzaklık (m)', 'dist_to_goal'),
               ('Min engel mesafe (m)', 'min_obs_dist'),
               ('Süre (s)', 'duration')]
    if summary['strict_available']:
        metrics += [
            ('Min engel mesafe — strict (m)', 'min_obstacle_distance'),
            ('Max y sapması (m)', 'max_y_deviation'),
            ('|final y hatası| (m)', 'final_y_error_abs'),
            ('State geçiş sayısı', 'state_transition_count'),
            ('Stuck süresi (s)', 'stuck_time'),
            ('cmd_vel oscillation [0..1]', 'cmd_vel_oscillation_score'),
            ('Trial süresi (s)', 'trial_duration'),
        ]
    if summary['trial_start_available']:
        metrics += [
            ('Trial start x (m)', 'trial_start_x'),
            ('Trial start y (m)', 'trial_start_y'),
        ]
    for label, key in metrics:
        s = summary[key]
        if s is None:
            print(f"| {label} | — | — | — | — |")
        else:
            print(f"| {label} | {s['mean']:.2f} | {s['std']:.2f} | "
                  f"{s['min']:.2f} | {s['max']:.2f} |")
    print()
    if summary['strict_available']:
        print(f"- **STRICT PASS oranı:** {summary['strict_passes']}/"
              f"{summary['total']} (**{summary['strict_pass_rate']:.1f}%**)")
        print()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('csv_path')
    ap.add_argument('--markdown', action='store_true')
    args = ap.parse_args()

    with open(args.csv_path) as f:
        rows = list(csv.DictReader(f))
    if not rows:
        print(f"Boş CSV: {args.csv_path}", file=sys.stderr)
        sys.exit(1)

    summary = summarize(rows)
    if args.markdown:
        print_markdown(summary, args.csv_path)
    else:
        print_text(summary, args.csv_path)


if __name__ == '__main__':
    main()
