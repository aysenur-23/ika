"""IKA Tez Benchmark Tablo + Plot Jeneratoru.

raw_runs.csv -> ozet markdown tablo + LaTeX tablo + matplotlib plotlar.

Cikti:
  benchmarks/results/summary.md     - tezde direkt kullanabilirsin
  benchmarks/results/summary.tex    - \\input{} ile LaTeX'e dahil
  benchmarks/results/plots/*.png    - grafik dosyalari

Plot turleri:
  - per_scenario_metric_bar.png     - mod x metric bar grafigi (her senaryo)
  - all_trajectories_per_scenario   - tum trial'larin yorungeleri ust uste
  - success_rate_heatmap.png        - senaryo x mod basari haritasi
  - duration_boxplot.png            - sure dagilimi kutu grafigi

Kullanim:
  python3 benchmarks/render_tables.py
"""
from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from statistics import mean, stdev

try:
    import yaml
    import numpy as np
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
except ImportError as e:
    raise SystemExit(f"Eksik: {e}. pip install pyyaml numpy matplotlib")


BENCHMARK_DIR = Path(__file__).resolve().parent
RESULTS_DIR = BENCHMARK_DIR / "results"


def read_runs(csv_path: Path) -> list[dict]:
    if not csv_path.exists():
        return []
    with open(csv_path) as f:
        return list(csv.DictReader(f))


def group_by(rows: list[dict], *keys: str) -> dict:
    grouped = defaultdict(list)
    for r in rows:
        k = tuple(r[k] for k in keys)
        grouped[k].append(r)
    return grouped


def stats(values: list[float]) -> dict:
    if not values:
        return {'mean': 0.0, 'std': 0.0, 'min': 0.0, 'max': 0.0, 'n': 0}
    m = mean(values)
    s = stdev(values) if len(values) > 1 else 0.0
    return {'mean': m, 'std': s, 'min': min(values), 'max': max(values),
            'n': len(values)}


def render_markdown_summary(rows: list[dict], out_path: Path):
    """senaryo x mod ozet markdown tablosu."""
    by_combo = group_by(rows, 'scenario_id', 'mode_id')

    scenarios = sorted({r['scenario_id'] for r in rows})
    modes = sorted({r['mode_id'] for r in rows})

    out = []
    out.append("# IKA Tez Benchmark Sonuclari\n")
    out.append(f"Toplam koşum: **{len(rows)}**\n")
    out.append(f"Senaryo: {len(scenarios)} · Mod: {len(modes)}\n\n")
    out.append("## Senaryo × Mod Ozet Tablosu\n\n")
    out.append("Her hücre: success_rate% · ortalama_süre±std · ortalama_yol_uzunluğu\n\n")

    # Tablo başlık
    header = ["Senaryo"] + modes
    out.append("| " + " | ".join(header) + " |")
    out.append("|" + "|".join(["---"] * len(header)) + "|")

    for sid in scenarios:
        row = [sid]
        for mid in modes:
            combo_runs = by_combo.get((sid, mid), [])
            if not combo_runs:
                row.append("—")
                continue
            success_rate = sum(1 for r in combo_runs if r['success'].lower() == 'true') / len(combo_runs) * 100
            durs = [float(r['duration_s']) for r in combo_runs]
            plens = [float(r['path_length_m']) for r in combo_runs]
            d_stats = stats(durs)
            p_stats = stats(plens)
            cell = (f"**{success_rate:.0f}%** · "
                    f"{d_stats['mean']:.1f}±{d_stats['std']:.1f}s · "
                    f"{p_stats['mean']:.2f}m")
            row.append(cell)
        out.append("| " + " | ".join(row) + " |")

    out.append("\n## Metrik Detaylari (Mod Bazli)\n")
    out.append("| Mod | Senaryo | Success% | Süre ± std (s) | Yol (m) | Min Clearance (m) | Curvature (rad/m) | Avg Speed (m/s) |")
    out.append("|---|---|---|---|---|---|---|---|")
    for mid in modes:
        for sid in scenarios:
            combo_runs = by_combo.get((sid, mid), [])
            if not combo_runs:
                continue
            sr = sum(1 for r in combo_runs if r['success'].lower() == 'true') / len(combo_runs) * 100
            durs = [float(r['duration_s']) for r in combo_runs]
            plens = [float(r['path_length_m']) for r in combo_runs]
            clears = [float(r['min_clearance_m']) for r in combo_runs
                      if r['min_clearance_m'] not in ('inf', 'Infinity')]
            curvs = [float(r['mean_abs_curvature']) for r in combo_runs]
            spds = [float(r['avg_speed_mps']) for r in combo_runs]
            ds, ps, cs = stats(durs), stats(plens), stats(clears)
            cv, sp = stats(curvs), stats(spds)
            out.append(
                f"| {mid} | {sid} | {sr:.0f}% | "
                f"{ds['mean']:.2f}±{ds['std']:.2f} | "
                f"{ps['mean']:.2f}±{ps['std']:.2f} | "
                f"{cs['mean']:.3f}±{cs['std']:.3f} | "
                f"{cv['mean']:.4f}±{cv['std']:.4f} | "
                f"{sp['mean']:.3f}±{sp['std']:.3f} |"
            )

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(out))
    print(f"[render] {out_path}")


def render_latex_summary(rows: list[dict], out_path: Path):
    """Tezdeki bir bolume \\input{} ile dahil edilebilir LaTeX tablosu."""
    by_combo = group_by(rows, 'scenario_id', 'mode_id')
    scenarios = sorted({r['scenario_id'] for r in rows})
    modes = sorted({r['mode_id'] for r in rows})

    lines = []
    lines.append(r"% IKA Tez Benchmark Sonuclari — auto-generated")
    lines.append(r"\begin{table}[h]")
    lines.append(r"\centering")
    lines.append(r"\caption{Senaryo×Mod basari ve sure ozetleri (5 trial ortalama±std).}")
    lines.append(r"\label{tab:ika_benchmark_summary}")
    lines.append(r"\begin{tabular}{l" + "c" * len(modes) + "}")
    lines.append(r"\toprule")
    lines.append("Senaryo & " + " & ".join(modes) + r" \\")
    lines.append(r"\midrule")
    for sid in scenarios:
        cells = [sid.replace('_', r'\_')]
        for mid in modes:
            combo = by_combo.get((sid, mid), [])
            if not combo:
                cells.append("—")
                continue
            sr = sum(1 for r in combo if r['success'].lower() == 'true') / len(combo) * 100
            ds = stats([float(r['duration_s']) for r in combo])
            cells.append(f"{sr:.0f}\\% · {ds['mean']:.1f}$\\pm${ds['std']:.1f}\\,s")
        lines.append(" & ".join(cells) + r" \\")
    lines.append(r"\bottomrule")
    lines.append(r"\end{tabular}")
    lines.append(r"\end{table}")

    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text("\n".join(lines))
    print(f"[render] {out_path}")


def plot_trajectories_per_scenario(rows: list[dict], traj_dir: Path,
                                   plot_dir: Path):
    """Her senaryo icin: tum trial'larin yorungesi + obstacles + goal."""
    scenarios = sorted({r['scenario_id'] for r in rows})
    plot_dir.mkdir(parents=True, exist_ok=True)
    mode_colors = {'avoider': '#1f77b4', 'nav2_dwb': '#2ca02c', 'nav2_mppi': '#d62728'}

    for sid in scenarios:
        fig, ax = plt.subplots(figsize=(10, 5))
        first = True
        goal_xy = None
        obs_xy = None
        for npz_path in sorted(traj_dir.glob(f"{sid}_*.npz")):
            data = np.load(str(npz_path), allow_pickle=True)
            traj = data['trajectory']
            md = json.loads(str(data['metadata']))
            mid = md['mode_id']
            color = mode_colors.get(mid, '#888')
            if traj.shape[0] > 0:
                ax.plot(traj[:, 1], traj[:, 2], '-', color=color, alpha=0.5,
                        label=mid if first else None)
            if first:
                goal_xy = data['goal']
                obs_xy = data['obstacles']
                first = False
        if goal_xy is not None:
            ax.plot(goal_xy[0], goal_xy[1], 'r*', markersize=15, label='goal')
        if obs_xy is not None and obs_xy.shape[0] > 0:
            ax.plot(obs_xy[:, 0], obs_xy[:, 1], 'kx', markersize=10, label='engel')
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
        ax.set_title(f"Senaryo: {sid} — yorungeler (tum mod × trial)")
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend(loc='best', fontsize=9)
        out = plot_dir / f'traj_{sid}.png'
        fig.tight_layout()
        fig.savefig(out, dpi=140)
        plt.close(fig)
        print(f"[plot] {out}")


def plot_success_heatmap(rows: list[dict], plot_dir: Path):
    scenarios = sorted({r['scenario_id'] for r in rows})
    modes = sorted({r['mode_id'] for r in rows})
    grid = np.zeros((len(scenarios), len(modes)))
    by_combo = group_by(rows, 'scenario_id', 'mode_id')
    for i, sid in enumerate(scenarios):
        for j, mid in enumerate(modes):
            combo = by_combo.get((sid, mid), [])
            if combo:
                grid[i, j] = sum(1 for r in combo if r['success'].lower() == 'true') / len(combo) * 100

    fig, ax = plt.subplots(figsize=(8, max(4, len(scenarios) * 0.4)))
    im = ax.imshow(grid, cmap='RdYlGn', vmin=0, vmax=100, aspect='auto')
    ax.set_xticks(range(len(modes))); ax.set_xticklabels(modes)
    ax.set_yticks(range(len(scenarios))); ax.set_yticklabels(scenarios)
    for i in range(len(scenarios)):
        for j in range(len(modes)):
            ax.text(j, i, f'{grid[i,j]:.0f}%', ha='center', va='center',
                    color='black', fontsize=9)
    ax.set_title('Senaryo × Mod Basari Orani (%)')
    plt.colorbar(im, ax=ax, label='success %')
    plot_dir.mkdir(parents=True, exist_ok=True)
    out = plot_dir / 'success_heatmap.png'
    fig.tight_layout(); fig.savefig(out, dpi=140); plt.close(fig)
    print(f"[plot] {out}")


def plot_duration_boxplot(rows: list[dict], plot_dir: Path):
    by_mode = group_by(rows, 'mode_id')
    modes = sorted(by_mode.keys())
    data = [[float(r['duration_s']) for r in by_mode[m] if r['success'].lower() == 'true']
            for m in modes]
    fig, ax = plt.subplots(figsize=(8, 5))
    ax.boxplot(data, labels=[m[0] for m in modes])
    ax.set_ylabel('Sure (s)')
    ax.set_title('Hedefe ulasma suresi - mod bazli (sadece success run)')
    ax.grid(True, axis='y', alpha=0.3)
    plot_dir.mkdir(parents=True, exist_ok=True)
    out = plot_dir / 'duration_boxplot.png'
    fig.tight_layout(); fig.savefig(out, dpi=140); plt.close(fig)
    print(f"[plot] {out}")


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--config', type=Path,
                   default=BENCHMARK_DIR / 'scenarios.yaml')
    args = p.parse_args()

    cfg = yaml.safe_load(open(args.config))
    csv_path = RESULTS_DIR / cfg['output']['raw_csv']
    rows = read_runs(csv_path)
    if not rows:
        print(f"Sonuc CSV bos veya yok: {csv_path}")
        print("Once 'python3 benchmarks/run_benchmark.py' kos.")
        return

    print(f"[render] {len(rows)} run okundu: {csv_path}")
    render_markdown_summary(rows, RESULTS_DIR / cfg['output']['summary_md'])
    render_latex_summary(rows, RESULTS_DIR / cfg['output']['summary_tex'])
    plot_trajectories_per_scenario(
        rows,
        RESULTS_DIR / 'trajectories',
        RESULTS_DIR / cfg['output']['plots_dir'])
    plot_success_heatmap(rows, RESULTS_DIR / cfg['output']['plots_dir'])
    plot_duration_boxplot(rows, RESULTS_DIR / cfg['output']['plots_dir'])
    print("\n[render] BITTI. tezde:")
    print(f"  - Markdown ozet : {RESULTS_DIR / cfg['output']['summary_md']}")
    print(f"  - LaTeX tablo  : {RESULTS_DIR / cfg['output']['summary_tex']}")
    print(f"  - Grafikler    : {RESULTS_DIR / cfg['output']['plots_dir']}")


if __name__ == '__main__':
    main()
