#!/usr/bin/env python3
"""
Random-Params Analysis — 200 cities (worker_hyperparams=random)
Reads summary .txt files from each outputs/random-params/<strategy>-<factor>/ subdir.
"""

import glob
import os

OUTPUTS_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE    = os.path.join(OUTPUTS_DIR, "analysis_result.txt")

BL_AVG  = 84.291200
BL_BEST = 82.606487
BL_TIME = 25.55
BL_STD  = 0.558229


def read_summary(path):
    info = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if "=" in line:
                k, v = line.split("=", 1)
                info[k.strip()] = v.strip()
    return info


STRATEGY_ORDER = {"random": 0, "topk": 1, "rank": 2, "pullcount": 3}


def load_cases():
    rows = []
    for subdir in sorted(glob.glob(os.path.join(OUTPUTS_DIR, "*"))):
        if not os.path.isdir(subdir):
            continue
        name  = os.path.basename(subdir)
        # format: {strategy}-{factor}  e.g. rank-1.0, pullcount-0.6
        parts = name.rsplit("-", 1)
        if len(parts) != 2:
            continue
        try:
            strategy = parts[0]
            factor   = float(parts[1])
        except ValueError:
            continue

        summaries = sorted(glob.glob(os.path.join(subdir, "*-summary.txt")))
        if not summaries:
            print(f"  WARNING: no summary file in {subdir}")
            continue

        info = read_summary(summaries[-1])
        rows.append({
            "strategy": strategy,
            "factor":   factor,
            "avg":  float(info["average_result"]),
            "best": float(info["best_result"]),
            "std":  float(info["combined_std_result"]),
            "time": float(info["average_total_time_sec"]),
        })

    rows.sort(key=lambda r: (r["factor"], STRATEGY_ORDER.get(r["strategy"], 99)))
    return rows


def s(v):
    return f"{v:+.2f}"


def main():
    rows = load_cases()
    if not rows:
        print("No data found.")
        return

    sep = "-" * 118
    out = []

    # ── Header ──────────────────────────────────────────────────────────────
    out.append("Random-Params Analysis — 200 cities  (worker_hyperparams=random)")
    out.append("=" * 66)
    out.append("")
    out.append("Baseline (sequential C/C++, 200 customers):")
    out.append(f"  best_result            = {BL_BEST}")
    out.append(f"  average_result         = {BL_AVG}")
    out.append(f"  average_total_time_sec = {BL_TIME}")
    out.append(f"  combined_std_result    = {BL_STD}")
    out.append("")
    out.append("best_diff% = (best_result - BL_BEST) / BL_BEST * 100")
    out.append("avg_diff%  = (avg_result  - BL_AVG)  / BL_AVG  * 100")
    out.append("time_diff% = (avg_time    - BL_TIME) / BL_TIME * 100")
    out.append("Negative = better than baseline.")
    out.append("")

    # ── Main table (grouped by strategy) ────────────────────────────────────
    out.append("Full results  (sorted by factor / strategy):")
    out.append(sep)
    out.append(
        f"  {'factor':<8} {'strategy':<12} "
        f"{'best_result':>12} {'avg_result':>12} {'std':>10} {'avg_time':>10} "
        f"{'best_diff%':>11} {'avg_diff%':>10} {'CV%':>6} {'time_diff%':>11}"
    )
    out.append(sep)

    prev_factor = None
    for r in rows:
        if prev_factor is not None and r["factor"] != prev_factor:
            out.append(sep)
        prev_factor = r["factor"]

        bd = (r["best"] - BL_BEST) / BL_BEST * 100
        rd = (r["avg"]  - BL_AVG)  / BL_AVG  * 100
        td = (r["time"] - BL_TIME) / BL_TIME * 100
        cv = r["std"] / r["avg"] * 100
        out.append(
            f"  {r['factor']:<8.1f} {r['strategy']:<12} "
            f"{r['best']:>12.6f} {r['avg']:>12.6f} {r['std']:>10.6f} {r['time']:>10.2f} "
            f"{s(bd):>11} {s(rd):>10} {cv:>6.2f} {s(td):>11}"
        )

    # ── TB group (average across all factors per strategy) ───────────────────
    out.append(sep)
    strategies = sorted(set(r["strategy"] for r in rows), key=lambda x: STRATEGY_ORDER.get(x, 99))
    for strat in strategies:
        group = [r for r in rows if r["strategy"] == strat]
        tb_best = sum(r["best"] for r in group) / len(group)
        tb_avg  = sum(r["avg"]  for r in group) / len(group)
        tb_std  = sum(r["std"]  for r in group) / len(group)
        tb_time = sum(r["time"] for r in group) / len(group)
        bd = (tb_best - BL_BEST) / BL_BEST * 100
        rd = (tb_avg  - BL_AVG)  / BL_AVG  * 100
        td = (tb_time - BL_TIME) / BL_TIME * 100
        cv = tb_std / tb_avg * 100
        out.append(
            f"  {'TB':<8} {strat:<12} "
            f"{tb_best:>12.6f} {tb_avg:>12.6f} {tb_std:>10.6f} {tb_time:>10.2f} "
            f"{s(bd):>11} {s(rd):>10} {cv:>6.2f} {s(td):>11}"
        )

    out.append(sep)
    out.append("")

    # ── Notes ────────────────────────────────────────────────────────────────
    best_row    = min(rows, key=lambda r: r["best"])
    fastest_row = min(rows, key=lambda r: r["time"])

    def cfg(r):
        return f"strategy={r['strategy']} / factor={r['factor']:.1f}"

    out.append("Notes:")
    out.append(
        f"- Best best_result:  {cfg(best_row)}"
        f" → {best_row['best']:.6f}"
        f" (best_diff% = {s((best_row['best'] - BL_BEST) / BL_BEST * 100)}%)"
    )
    out.append(
        f"- Fastest:          {cfg(fastest_row)}"
        f" → {fastest_row['time']:.2f}s"
        f" (time_diff% = {s((fastest_row['time'] - BL_TIME) / BL_TIME * 100)}%)"
    )

    text = "\n".join(out) + "\n"
    with open(OUT_FILE, "w") as f:
        f.write(text)

    print(text)
    print(f"Saved: {OUT_FILE}")


if __name__ == "__main__":
    main()
