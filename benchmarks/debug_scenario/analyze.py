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


def summarize(rows):
    total = len(rows)
    status_counts = Counter(r['status'] for r in rows)
    passes = status_counts.get('PASS', 0)

    def num(rs, key, valid_only=True):
        vals = []
        for r in rs:
            try:
                v = float(r[key])
                if valid_only and (math.isnan(v) or v < -100):
                    continue
                vals.append(v)
            except (ValueError, KeyError):
                continue
        return vals

    dist = num(rows, 'dist_to_goal')
    min_obs = num(rows, 'min_obs_dist')
    dur = num(rows, 'duration')

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
    for label, key in [('Hedefe uzaklık (m)', 'dist_to_goal'),
                       ('Min engel mesafe (m)', 'min_obs_dist'),
                       ('Süre (s)', 'duration')]:
        s = summary[key]
        if s is None:
            print(f"| {label} | — | — | — | — |")
        else:
            print(f"| {label} | {s['mean']:.2f} | {s['std']:.2f} | "
                  f"{s['min']:.2f} | {s['max']:.2f} |")
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
