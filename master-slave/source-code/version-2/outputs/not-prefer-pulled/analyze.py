#!/usr/bin/env python3
"""
Not-Prefer-Pulled Analysis — 200 cities
Reads summary .txt files from each outputs/not-prefer-pulled/<ai>-<seg>-<pool>/ subdir.
"""

import glob
import os

OUTPUTS_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE    = os.path.join(OUTPUTS_DIR, "analysis_result.txt")

BL_AVG  = 82.960189
BL_BEST = 81.625510
BL_TIME = 21.54
BL_STD  = 0.636281


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
        parts = name.split("-")
        if len(parts) != 3:
            continue
        try:
            ai   = int(parts[0])
            seg  = int(parts[1])
            pool = float(parts[2])
        except ValueError:
            continue

        summaries = sorted(glob.glob(os.path.join(subdir, "*-summary.txt")))
        if not summaries:
            print(f"  WARNING: no summary file in {subdir}")
            continue

        info = read_summary(summaries[-1])
        rows.append({
            "ai":   ai,
            "seg":  seg,
            "pool": pool,
            "avg":  float(info["average_result"]),
            "best": float(info["best_result"]),
            "std":  float(info["combined_std_result"]),
            "time": float(info["average_total_time_sec"]),
        })

    rows.sort(key=lambda r: (r["ai"], r["seg"], r["pool"]))
    return rows


def s(v):
    return f"{v:+.2f}"



def main():
    rows = load_cases()
    if not rows:
        print("No data found.")
        return

    sep  = "-" * 120
    out  = []

    # ── Header ──────────────────────────────────────────────────────────────
    out.append("Not-Prefer-Pulled Analysis — 200 cities")
    out.append("=====================================")
    out.append("")
    out.append(f"Baseline (config 10-4-0.06):")
    out.append(f"  best_result            = {BL_BEST}")
    out.append(f"  average_result         = {BL_AVG}")
    out.append(f"  average_total_time_sec = {BL_TIME}")
    out.append(f"  combined_std_result    = {BL_STD}")
    out.append("")
    out.append("best_diff%   = (best_result  - BL_BEST) / BL_BEST * 100")
    out.append("result_diff% = (avg_result   - BL_AVG)  / BL_AVG  * 100")
    out.append("time_diff%   = (avg_time_sec - BL_TIME) / BL_TIME * 100")
    out.append("Negative = better than baseline.")
    out.append("")

    # ── Main table (sorted by ai, seg, pool; grouped by ai+seg) ────────────
    out.append("Full results  (sorted by ai / seg / pool):")
    out.append(sep)
    out.append(
        f"  {'ai':<6} {'seg':<6} {'pool':<8} "
        f"{'best_result':>12} {'avg_result':>12} {'std':>10} {'avg_time':>10} "
        f"{'best_diff%':>11} {'avg_diff%':>10} {'CV%':>6} {'time_diff%':>11}"
    )
    out.append(sep)

    prev_group = None
    for r in rows:
        group = (r["ai"], r["seg"])
        if prev_group is not None and group != prev_group:
            out.append(sep)
        prev_group = group

        bd = (r["best"] - BL_BEST) / BL_BEST * 100
        rd = (r["avg"]  - BL_AVG)  / BL_AVG  * 100
        td = (r["time"] - BL_TIME) / BL_TIME * 100
        cv = r["std"] / r["avg"] * 100
        out.append(
            f"  {r['ai']:<6} {r['seg']:<6} {r['pool']:<8.2f} "
            f"{r['best']:>12.6f} {r['avg']:>12.6f} {r['std']:>10.6f} {r['time']:>10.2f} "
            f"{s(bd):>11} {s(rd):>10} {cv:>6.2f} {s(td):>11}"
        )

    out.append(sep)
    out.append("")

    # ── Notes ────────────────────────────────────────────────────────────────
    best_row    = min(rows, key=lambda r: r["best"])
    fastest_row = min(rows, key=lambda r: r["time"])

    def cfg(r):
        return f"iters={r['ai']} / segs={r['seg']} / pool={r['pool']:.2f}"

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
    out.append(
        f"- Baseline:         iters=10 / segs=4 / pool=0.06  (all diff% = +0.00%)"
    )

    text = "\n".join(out) + "\n"
    with open(OUT_FILE, "w") as f:
        f.write(text)

    print(text)
    print(f"Saved: {OUT_FILE}")


if __name__ == "__main__":
    main()
