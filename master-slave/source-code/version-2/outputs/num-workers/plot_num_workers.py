#!/usr/bin/env python3
"""
Plot num-workers results from outputs/num-workers/<strategy>-factor<factor>-np<np>/
"""

import glob
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUTPUTS_DIR = os.path.dirname(os.path.abspath(__file__))

BL_AVG  = 84.291200
BL_BEST = 82.606487
BL_TIME = 25.55


def read_summary(path):
    info = {}
    with open(path) as f:
        for line in f:
            if '=' in line:
                k, v = line.strip().split('=', 1)
                info[k.strip()] = v.strip()
    return info


def load_rows():
    rows = []
    for subdir in sorted(glob.glob(os.path.join(OUTPUTS_DIR, '*'))):
        if not os.path.isdir(subdir):
            continue
        name  = os.path.basename(subdir)
        parts = name.split('-')
        if len(parts) != 3:
            continue
        try:
            strategy = parts[0]
            factor   = float(parts[1].replace('factor', ''))
            np_val   = int(parts[2].replace('np', ''))
        except ValueError:
            continue

        summaries = sorted(glob.glob(os.path.join(subdir, '*-summary.txt')))
        if not summaries:
            continue
        info = read_summary(summaries[-1])
        rows.append({
            'np':       np_val,
            'strategy': strategy,
            'factor':   factor,
            'avg_r':    float(info['average_result']),
            'avg_t':    float(info['average_total_time_sec']),
            'best':     float(info['best_result']),
            'std':      float(info['combined_std_result']),
        })
    return rows


def annotate(ax, nps, vals, color, dx=0, dy=8, fmt='{:.2f}'):
    for x, y in zip(nps, vals):
        ax.annotate(fmt.format(y), (x, y), textcoords='offset points',
                    xytext=(dx, dy), ha='center', fontsize=7.5, color=color)


def plot(rows, strategy, factor, out_path):
    subset = sorted(
        [r for r in rows if r['strategy'] == strategy and r['factor'] == factor],
        key=lambda r: r['np'],
    )
    if not subset:
        print(f'  No data for strategy={strategy} factor={factor}')
        return

    nps   = [r['np']    for r in subset]
    avg_r = [r['avg_r'] for r in subset]
    best  = [r['best']  for r in subset]
    avg_t = [r['avg_t'] for r in subset]
    stds  = [r['std']   for r in subset]

    fig, ax_r = plt.subplots(figsize=(11, 5))
    ax_t = ax_r.twinx()

    l1, = ax_r.plot(nps, avg_r, marker='o', linewidth=2, markersize=6,
                    color='#2196F3', label='avg_result')
    l2, = ax_r.plot(nps, best,  marker='s', linewidth=2, markersize=6,
                    color='#4CAF50', label='best_result')
    bl1 = ax_r.axhline(BL_AVG,  color='#2196F3', linewidth=1.2, linestyle='--',
                        label=f'seq avg ({BL_AVG:.2f})')
    bl2 = ax_r.axhline(BL_BEST, color='#4CAF50', linewidth=1.2, linestyle='--',
                        label=f'seq best ({BL_BEST:.2f})')

    annotate(ax_r, nps, avg_r, '#2196F3', dx=-12, dy=6)
    annotate(ax_r, nps, best,  '#4CAF50', dx=12,  dy=-14)

    ax_r.set_ylabel('Result', fontsize=10)
    ax_r.set_xlabel('num workers', fontsize=10)
    ax_r.set_xticks(nps)
    ax_r.set_xticklabels([str(n - 1) for n in nps])
    ax_r.grid(True, alpha=0.25, linestyle='--')

    l3, = ax_t.plot(nps, avg_t, marker='^', linewidth=2, markersize=6,
                    color='#FF9800', label='avg_time_sec')
    bl3 = ax_t.axhline(BL_TIME, color='#FF9800', linewidth=1.2, linestyle='--',
                        label=f'seq time ({BL_TIME:.2f}s)')
    annotate(ax_t, nps, avg_t, '#FF9800', dx=0, dy=8)
    ax_t.set_ylabel('Time (sec)', fontsize=10, color='#FF9800')
    ax_t.tick_params(axis='y', labelcolor='#FF9800')

    ax_r.legend(handles=[l1, l2, l3, bl1, bl2, bl3], fontsize=8.5,
                loc='upper center', bbox_to_anchor=(0.5, -0.13), ncol=3, frameon=True)

    # Prevent BL_TIME (right axis) from visually overlapping BL_AVG / BL_BEST (left axis).
    # Both axes share the same plot area so their visual y-positions are directly comparable.
    r_ymin, r_ymax = ax_r.get_ylim()
    t_ymin, t_ymax = ax_t.get_ylim()
    r_range   = r_ymax - r_ymin
    frac_avg  = (BL_AVG  - r_ymin) / r_range
    frac_best = (BL_BEST - r_ymin) / r_range
    frac_time = (BL_TIME - t_ymin) / (t_ymax - t_ymin)
    if abs(frac_time - frac_avg) < 0.12 or abs(frac_time - frac_best) < 0.12:
        # Push BL_TIME to 65 % of chart height by expanding ax_t upward
        TARGET      = 0.65
        new_t_range = (BL_TIME - t_ymin) / TARGET
        new_t_ymax  = max(t_ymin + new_t_range, max(avg_t) * 1.05)
        ax_t.set_ylim(t_ymin, new_t_ymax)

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, bbox_inches='tight')
    plt.close()
    print(f'Saved: {out_path}')


if __name__ == '__main__':
    rows = load_rows()

    # Collect unique (strategy, factor) combos
    combos = sorted(set((r['strategy'], r['factor']) for r in rows))
    for strategy, factor in combos:
        fname = f'num_workers_chart_{strategy}_factor{factor:.1f}.png'
        plot(rows, strategy, factor, os.path.join(OUTPUTS_DIR, fname))
