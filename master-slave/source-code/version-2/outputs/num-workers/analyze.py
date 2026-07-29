#!/usr/bin/env python3
"""
Num-Workers Analysis — 200 cities (strategy=rank, factor=1.0)
Reads summary .txt files from each outputs/num-workers/<strategy>-factor<factor>-np<np>/ subdir.
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


def load_cases():
    rows = []
    for subdir in sorted(glob.glob(os.path.join(OUTPUTS_DIR, "*"))):
        if not os.path.isdir(subdir):
            continue
        name  = os.path.basename(subdir)
        # format: {strategy}-factor{factor}-np{np}  e.g. rank-factor1.0-np10
        parts = name.split("-")
        if len(parts) != 3:
            continue
        try:
            strategy = parts[0]
            factor   = float(parts[1].replace("factor", ""))
            np_val   = int(parts[2].replace("np", ""))
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
            "np":       np_val,
            "avg":  float(info["average_result"]),
            "best": float(info["best_result"]),
            "std":  float(info["combined_std_result"]),
            "time": float(info["average_total_time_sec"]),
        })

    rows.sort(key=lambda r: (r["strategy"], r["factor"], r["np"]))
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
    out.append("Num-Workers Analysis — 200 cities  (strategy=rank, factor=1.0)")
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

    # ── Main table (grouped by strategy+factor) ──────────────────────────────
    out.append("Full results  (sorted by strategy / factor / np):")
    out.append(sep)
    out.append(
        f"  {'strategy':<12} {'factor':<8} {'np':>4}  "
        f"{'best_result':>12} {'avg_result':>12} {'std':>10} {'avg_time':>10} "
        f"{'best_diff%':>11} {'avg_diff%':>10} {'CV%':>6} {'time_diff%':>11}"
    )
    out.append(sep)

    prev_group = None
    for r in rows:
        group = (r["strategy"], r["factor"])
        if prev_group is not None and group != prev_group:
            out.append(sep)
        prev_group = group

        bd = (r["best"] - BL_BEST) / BL_BEST * 100
        rd = (r["avg"]  - BL_AVG)  / BL_AVG  * 100
        td = (r["time"] - BL_TIME) / BL_TIME * 100
        cv = r["std"] / r["avg"] * 100
        out.append(
            f"  {r['strategy']:<12} {r['factor']:<8.1f} {r['np']:>4}  "
            f"{r['best']:>12.6f} {r['avg']:>12.6f} {r['std']:>10.6f} {r['time']:>10.2f} "
            f"{s(bd):>11} {s(rd):>10} {cv:>6.2f} {s(td):>11}"
        )

    out.append(sep)
    out.append("")

    # ── Notes ────────────────────────────────────────────────────────────────
    best_row    = min(rows, key=lambda r: r["best"])
    fastest_row = min(rows, key=lambda r: r["time"])

    def cfg(r):
        return f"strategy={r['strategy']} / factor={r['factor']:.1f} / np={r['np']}"

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
