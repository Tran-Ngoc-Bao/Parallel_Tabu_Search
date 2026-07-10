import glob
import os
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BL_AVG  = 84.2912
BL_BEST = 82.606487
BL_TIME = 25.55

SUMMARY_DIR = os.path.dirname(os.path.abspath(__file__))

CONFIGS = [
    {
        'strategy': 'topk', 'factor': '6', 'rand': 'true',
        'out': os.path.join(SUMMARY_DIR, 'num_workers_chart_factor6.png'),
    },
    {
        'strategy': 'rank', 'factor': '12', 'rand': 'false',
        'out': os.path.join(SUMMARY_DIR, 'num_workers_chart_factor12.png'),
    },
]


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
    for sf in sorted(glob.glob(os.path.join(SUMMARY_DIR, '*-summary.txt'))):
        info = read_summary(sf)
        rows.append({
            'np':       int(info['mpirun_np']),
            'strategy': info['elite_pull_strategy'],
            'factor':   info['min_pull_elites_per_worker_factor'],
            'rand':     info['randomize_worker_hyperparams'],
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


def plot_config(cfg, rows):
    subset = sorted(
        [r for r in rows
         if r['strategy'] == cfg['strategy']
         and r['factor'] == cfg['factor']
         and r['rand'] == cfg['rand']],
        key=lambda x: x['np'],
    )
    nps   = [r['np']    for r in subset]
    avg_r = [r['avg_r'] for r in subset]
    best  = [r['best']  for r in subset]
    avg_t = [r['avg_t'] for r in subset]

    fig, ax_r = plt.subplots(figsize=(11, 5))
    ax_t = ax_r.twinx()

    l1, = ax_r.plot(nps, avg_r, marker='o', linewidth=2, markersize=6,
                    color='#2196F3', label='avg_result')
    l2, = ax_r.plot(nps, best,  marker='s', linewidth=2, markersize=6,
                    color='#4CAF50', label='best_result')
    bl1 = ax_r.axhline(BL_AVG,  color='#2196F3', linewidth=1.2, linestyle='--',
                        label=f'baseline avg ({BL_AVG:.2f})')
    bl2 = ax_r.axhline(BL_BEST, color='#4CAF50', linewidth=1.2, linestyle='--',
                        label=f'baseline best ({BL_BEST:.2f})')

    annotate(ax_r, nps, avg_r, '#2196F3', dx=-12, dy=6)
    annotate(ax_r, nps, best,  '#4CAF50', dx=12,  dy=-14)

    ax_r.set_ylabel('Result (shorter = better)', fontsize=10)
    ax_r.set_xlabel('workers', fontsize=10)
    ax_r.set_xticks(nps)
    ax_r.set_xticklabels([str(n - 1) for n in nps])
    ax_r.grid(True, alpha=0.25, linestyle='--')

    l3, = ax_t.plot(nps, avg_t, marker='^', linewidth=2, markersize=6,
                    color='#FF9800', label='avg_time_sec')
    bl3 = ax_t.axhline(BL_TIME, color='#FF9800', linewidth=1.2, linestyle='--',
                        label=f'baseline time ({BL_TIME:.2f})')
    annotate(ax_t, nps, avg_t, '#FF9800', dx=0, dy=8)
    ax_t.set_ylabel('Time (sec)', fontsize=10, color='#FF9800')
    ax_t.tick_params(axis='y', labelcolor='#FF9800')

    ax_r.legend(handles=[l1, l2, l3, bl1, bl2, bl3], fontsize=8.5, loc='upper right')

    plt.tight_layout()
    plt.savefig(cfg['out'], dpi=150, bbox_inches='tight')
    plt.close()
    print(f"Saved: {cfg['out']}")


if __name__ == '__main__':
    rows = load_rows()
    for cfg in CONFIGS:
        plot_config(cfg, rows)
