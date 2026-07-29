#!/usr/bin/env python3
"""
Num-Customers Analysis (factor=1.0, np=10)
Reads summary .txt files from each outputs/num-customers/<customers>-<strategy>-factor<factor>/ subdir.
Baseline: sequential C/C++ algorithm per customer count.
"""

import glob
import math
import os

OUTPUTS_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE    = os.path.join(OUTPUTS_DIR, "analysis_result.txt")

# Sequential C/C++ baseline per customer count
SEQ_BASELINE = {
      6: {"best": 39.070569, "avg": 39.070570, "time": 0.02,  "std": 0.000000},
     10: {"best": 50.018212, "avg": 50.018212, "time": 0.05,  "std": 0.000000},
     12: {"best": 56.165487, "avg": 56.165487, "time": 0.08,  "std": 0.000000},
     20: {"best": 34.988516, "avg": 35.352293, "time": 0.12,  "std": 0.331927},
     50: {"best": 117.689245, "avg": 119.229045, "time": 1.34, "std": 0.699375},
    100: {"best": 122.450037, "avg": 123.057419, "time": 8.23, "std": 0.319962},
    200: {"best": 82.606487,  "avg": 84.291200,  "time": 25.55, "std": 0.558229},
}

STRATEGY_ORDER = {"random": 0, "topk": 1, "rank": 2, "pullcount": 3}


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
            customers = int(parts[0])
            strategy  = parts[1]
            factor    = float(parts[2].replace("factor", ""))
        except ValueError:
            continue

        summaries = sorted(glob.glob(os.path.join(subdir, "*-summary.txt")))
        if not summaries:
            print(f"  WARNING: no summary file in {subdir}")
            continue

        info    = read_summary(summaries[-1])
        std_val = float(info["combined_std_result"])
        rows.append({
            "customers": customers,
            "strategy":  strategy,
            "factor":    factor,
            "avg":  float(info["average_result"]),
            "best": float(info["best_result"]),
            "std":  0.0 if math.isnan(std_val) else std_val,
            "time": float(info["average_total_time_sec"]),
        })

    rows.sort(key=lambda r: (STRATEGY_ORDER.get(r["strategy"], 99), r["customers"]))
    return rows


def s(v):
    return f"{v:+.2f}"


def main():
    rows = load_cases()
    if not rows:
        print("No data found.")
        return

    W   = 118
    sep  = "-" * W
    sep2 = "-" * 72
    out  = []

    # ── Header ──────────────────────────────────────────────────────────────
    out.append("Num-Customers Analysis  (factor=1.0, np=10)")
    out.append("=" * 60)
    out.append("")
    out.append("Baseline: sequential C/C++ algorithm, per customer count.")
    out.append("")
    out.append("best_diff% = (par best - seq best) / seq best * 100")
    out.append("avg_diff%  = (par avg  - seq avg)  / seq avg  * 100")
    out.append("time_diff% = (par time - seq time) / seq time * 100")
    out.append("Negative = better than baseline.")
    out.append("")

    # ── Seq baseline table ───────────────────────────────────────────────────
    out.append("Sequential baseline:")
    out.append(sep2)
    out.append(f"  {'customers':>10}  {'seq_best':>12} {'seq_avg':>12} {'avg_time':>10} {'std':>10}")
    out.append(sep2)
    for c, bl in sorted(SEQ_BASELINE.items()):
        out.append(f"  {c:>10}  {bl['best']:>12.6f} {bl['avg']:>12.6f} {bl['time']:>10.2f} {bl['std']:>10.6f}")
    out.append(sep2)
    out.append("")

    # ── Main table ──────────────────────────────────────────────────────────
    out.append("Full results  (sorted by strategy / customers):")
    out.append(sep)
    out.append(
        f"  {'customers':>10} {'strategy':>10}  "
        f"{'best_result':>12} {'avg_result':>12} {'std':>10} {'avg_time':>10} {'CV%':>6}  "
        f"{'best_diff%':>11} {'avg_diff%':>10} {'time_diff%':>11}"
    )
    out.append(sep)

    prev_strat = None
    for r in rows:
        if prev_strat is not None and r["strategy"] != prev_strat:
            out.append(sep)
        prev_strat = r["strategy"]

        c  = r["customers"]
        cv = r["std"] / r["avg"] * 100 if r["avg"] > 0 else 0.0

        seq = SEQ_BASELINE.get(c)
        if seq:
            bd = (r["best"] - seq["best"]) / seq["best"] * 100
            rd = (r["avg"]  - seq["avg"])  / seq["avg"]  * 100
            td = (r["time"] - seq["time"]) / seq["time"] * 100
            diff_str = f"{s(bd):>11} {s(rd):>10} {s(td):>11}"
        else:
            diff_str = f"{'N/A':>11} {'N/A':>10} {'N/A':>11}"

        out.append(
            f"  {c:>10} {r['strategy']:>10}  "
            f"{r['best']:>12.6f} {r['avg']:>12.6f} {r['std']:>10.6f} {r['time']:>10.2f} {cv:>6.2f}  "
            f"{diff_str}"
        )

    out.append(sep)
    out.append("")

    # ── Notes ────────────────────────────────────────────────────────────────
    best_row    = min(rows, key=lambda r: r["best"])
    fastest_row = min(rows, key=lambda r: r["time"])

    def cfg(r):
        return f"customers={r['customers']} strategy={r['strategy']} factor={r['factor']:.1f}"

    out.append("Notes:")
    out.append(f"- Best best_result: {cfg(best_row)} → {best_row['best']:.6f}")
    seq_b = SEQ_BASELINE.get(best_row["customers"])
    if seq_b:
        out.append(f"    best_diff% = {s((best_row['best'] - seq_b['best']) / seq_b['best'] * 100)}%")

    out.append(f"- Fastest:          {cfg(fastest_row)} → {fastest_row['time']:.2f}s")
    seq_f = SEQ_BASELINE.get(fastest_row["customers"])
    if seq_f:
        out.append(f"    time_diff% = {s((fastest_row['time'] - seq_f['time']) / seq_f['time'] * 100)}%")

    text = "\n".join(out) + "\n"
    with open(OUT_FILE, "w") as f:
        f.write(text)

    print(text)
    print(f"Saved: {OUT_FILE}")


if __name__ == "__main__":
    main()
