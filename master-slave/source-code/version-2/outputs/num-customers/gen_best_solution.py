#!/usr/bin/env python3
"""
Generate best-solution-<strategy>.xlsx for outputs/num-customers/
One file per strategy; compares parallel (np=10, factor=1.0) vs sequential C/C++
and paper baseline for 50, 100, 200 customers.
"""

import csv
import glob
import os
from collections import defaultdict

import openpyxl
from openpyxl.styles import Alignment, Border, Font, PatternFill, Side
from openpyxl.utils import get_column_letter

SCRIPT_DIR    = os.path.dirname(os.path.abspath(__file__))
SEQ_STATS     = "/app/sequential-algorithm/cpp/statistics"
PAR_OUTPUTS   = SCRIPT_DIR
BASELINE_PATH = os.path.join(SCRIPT_DIR, "capacity-1400_baseline.xlsx")
PREFIXES      = [50, 100, 200]
SUFFIXES      = [1, 2, 3, 4]
MIDDLES       = [10, 20, 30, 40]

# ── helpers ──────────────────────────────────────────────────────────────────

def load_csv(path):
    rows = []
    with open(path) as f:
        for r in csv.DictReader(f):
            rows.append({
                "data_file": r["data_file"],
                "run":       int(r["run"]),
                "result":    float(r["result"]),
                "time":      float(r["timing_total_sec"]),
            })
    return rows


def find_csv(directory, prefix_str):
    files = sorted(glob.glob(os.path.join(directory, f"benchmark-{prefix_str}-*.csv")))
    if not files:
        raise FileNotFoundError(f"No CSV found in {directory} for prefix {prefix_str}")
    return files[-1]


def group_by_file(rows):
    d = defaultdict(list)
    for r in rows:
        d[r["data_file"]].append(r)
    return d


def seq_best(runs):
    best = min(runs, key=lambda r: r["result"])
    return best["result"], best["time"]


# ── data loading ─────────────────────────────────────────────────────────────

def detect_strategies():
    """Return sorted list of strategy names that have directories for all PREFIXES."""
    found = set()
    for subdir in glob.glob(os.path.join(PAR_OUTPUTS, "*")):
        name  = os.path.basename(subdir)
        parts = name.split("-")
        if len(parts) == 3:
            try:
                int(parts[0])      # customers prefix
                found.add(parts[1])  # strategy name
            except ValueError:
                pass
    return sorted(found)


def load_all(strategy):
    data = {}
    for n in PREFIXES:
        prefix_str = f"{n}."
        seq_csv = find_csv(SEQ_STATS, prefix_str)
        par_dir = os.path.join(PAR_OUTPUTS, f"{n}-{strategy}-factor1.0")
        try:
            par_csv = find_csv(par_dir, prefix_str)
        except FileNotFoundError:
            print(f"  WARNING: no CSV for {n} customers / strategy={strategy}, skipping")
            continue
        seq_by_file = group_by_file(load_csv(seq_csv))
        par_by_file = group_by_file(load_csv(par_csv))

        for df in sorted(seq_by_file):
            s_result, s_time = seq_best(seq_by_file[df])
            p_runs = par_by_file.get(df, [])
            better = sorted(
                [r for r in p_runs if r["result"] < s_result],
                key=lambda r: r["result"],
            )
            data[(n, df)] = {
                "seq_result": s_result,
                "seq_time":   s_time,
                "seq_runs":   sorted(seq_by_file[df], key=lambda r: r["run"]),
                "par_runs":   sorted(p_runs,           key=lambda r: r["run"]),
                "better":     better,
            }
    return data


# ── styling ───────────────────────────────────────────────────────────────────

HDR_FILL   = PatternFill("solid", fgColor="FF1F4E79")
HDR_FONT   = Font(color="FFFFFFFF", bold=True, size=10)
SUB_FILL   = PatternFill("solid", fgColor="FF2E75B6")
SUB_FONT   = Font(color="FFFFFFFF", bold=True, size=9)
GRP_FILL   = PatternFill("solid", fgColor="FFD6E4F0")
GRP_FONT   = Font(bold=True, size=9)
NUM_FONT   = Font(size=9)
FILE_FONT  = Font(bold=True, size=9)
SIDE       = Side(style="thin")
BORDER     = Border(left=SIDE, right=SIDE, top=SIDE, bottom=SIDE)
CENTER     = Alignment(horizontal="center", vertical="center", wrap_text=True)
LEFT       = Alignment(horizontal="left",   vertical="center")
RIGHT      = Alignment(horizontal="right",  vertical="center")

def hdr(ws, row, col, value, fill=HDR_FILL, font=HDR_FONT):
    c = ws.cell(row=row, column=col, value=value)
    c.fill, c.font, c.alignment, c.border = fill, font, CENTER, BORDER
    return c

def val(ws, row, col, value, font=None, align=None):
    c = ws.cell(row=row, column=col, value=value)
    c.border = BORDER
    c.font   = font or NUM_FONT
    c.alignment = align or RIGHT
    return c



# ── baseline loader ──────────────────────────────────────────────────────────

def load_baseline():
    """Return {problem: best_cost} from capacity-1400_baseline.xlsx."""
    wb = openpyxl.load_workbook(BASELINE_PATH)
    ws = wb.active
    best = {}
    for row in ws.iter_rows(min_row=2, values_only=True):
        prob = row[0]   # Problem column
        cost = row[11]  # Cost [minute] column
        if prob and cost is not None:
            if prob not in best or cost < best[prob]:
                best[prob] = cost
    return best


# ── summary sheet ─────────────────────────────────────────────────────────────

def build_summary(ws, data):
    # ── header rows ─────────────────────────────────────────────────────────
    # Set all cell properties BEFORE merging (openpyxl only renders top-left
    # cell's fill; writing to merged interior cells after merge_cells causes
    # white background corruption).
    hdr(ws, 1, 1, "data_file")
    hdr(ws, 2, 1, "")              # A2 — interior of A1:A2 merge
    hdr(ws, 1, 2, "Tuần tự")
    hdr(ws, 1, 3, "")              # C1 — interior of B1:C1 merge
    hdr(ws, 1, 4, "Song song")
    hdr(ws, 1, 5, "")              # E1 — interior of D1:E1 merge
    hdr(ws, 1, 6, "Số new best\nsolution")
    hdr(ws, 2, 6, "")              # F2 — interior of F1:F2 merge
    ws.merge_cells("A1:A2")
    ws.merge_cells("B1:C1")
    ws.merge_cells("D1:E1")
    ws.merge_cells("F1:F2")

    # Row 2 subheaders (B2:E2 are NOT inside any merged region)
    for col, label in [(2, "result"), (3, "timing_total_sec"),
                       (4, "result"), (5, "timing_total_sec")]:
        hdr(ws, 2, col, label, fill=SUB_FILL, font=SUB_FONT)

    ws.row_dimensions[1].height = 28
    ws.row_dimensions[2].height = 28
    ws.column_dimensions["A"].width = 12
    ws.column_dimensions["B"].width = 13
    ws.column_dimensions["C"].width = 13
    ws.column_dimensions["D"].width = 13
    ws.column_dimensions["E"].width = 13
    ws.column_dimensions["F"].width = 11

    # ── data rows ────────────────────────────────────────────────────────────
    cur_row = 3
    file_row_start = {}

    for n in PREFIXES:
        for mid in MIDDLES:
            for suf in SUFFIXES:
                df = f"{n}.{mid}.{suf}"
                key = (n, df)
                if key not in data:
                    continue
                d = data[key]
                if not d["better"]:
                    continue

                file_row_start[key] = cur_row
                n_better = len(d["better"])

                val(ws, cur_row, 1, df,                font=FILE_FONT, align=LEFT)
                val(ws, cur_row, 2, round(d["seq_result"], 6))
                val(ws, cur_row, 3, round(d["seq_time"], 3))
                val(ws, cur_row, 4, round(d["better"][0]["result"], 6))
                val(ws, cur_row, 5, round(d["better"][0]["time"],   3))
                val(ws, cur_row, 6, n_better)

                for i, b in enumerate(d["better"][1:], 1):
                    val(ws, cur_row + i, 4, round(b["result"], 6))
                    val(ws, cur_row + i, 5, round(b["time"],   3))
                    for col in range(1, 7):
                        ws.cell(row=cur_row + i, column=col).border = BORDER

                if n_better > 1:
                    ws.merge_cells(
                        start_row=cur_row, end_row=cur_row + n_better - 1,
                        start_column=1, end_column=1,
                    )
                    ws.merge_cells(
                        start_row=cur_row, end_row=cur_row + n_better - 1,
                        start_column=2, end_column=2,
                    )
                    ws.merge_cells(
                        start_row=cur_row, end_row=cur_row + n_better - 1,
                        start_column=3, end_column=3,
                    )
                    ws.merge_cells(
                        start_row=cur_row, end_row=cur_row + n_better - 1,
                        start_column=6, end_column=6,
                    )

                cur_row += n_better

    # ── right-side statistics ────────────────────────────────────────────────
    def getattr_df(df, attr):
        parts = df.split(".")   # e.g. ["50", "10", "1"]
        if attr == "suffix":
            return int(parts[2])
        if attr == "middle":
            return int(parts[1])
        if attr == "prefix":
            return int(parts[0])
        return None

    def new_bests_by_cross(mid_or_suf, val_, prefix):
        return sum(
            len(d["better"])
            for (n, df), d in data.items()
            if n == prefix and getattr_df(df, mid_or_suf) == val_
        )

    def instances_better_cross(mid_or_suf, val_, prefix):
        return sum(
            1 for (n, df), d in data.items()
            if n == prefix and getattr_df(df, mid_or_suf) == val_ and d["better"]
        )

    H = 8  # start column for stats

    r = 1
    # Cross-tab: Số new best solution by middle × prefix
    hdr(ws, r, H, "Số new best solution", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(start_row=r, end_row=r, start_column=H, end_column=H + len(PREFIXES))
    r += 1
    hdr(ws, r, H, "Nhóm trung tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for mid in MIDDLES:
        hdr(ws, r, H, f".{mid}.", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, new_bests_by_cross("middle", mid, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(new_bests_by_cross("middle", m, p) for m in MIDDLES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Cross-tab: Số instance tốt hơn by middle × prefix
    hdr(ws, r, H, "Số instance tốt hơn", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(start_row=r, end_row=r, start_column=H, end_column=H + len(PREFIXES))
    r += 1
    hdr(ws, r, H, "Nhóm trung tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for mid in MIDDLES:
        hdr(ws, r, H, f".{mid}.", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, instances_better_cross("middle", mid, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(instances_better_cross("middle", m, p) for m in MIDDLES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Cross-tab: Số new best solution by suffix × prefix
    hdr(ws, r, H, "Số new best solution", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):         # pre-fill interior merge cells
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(
        start_row=r, end_row=r,
        start_column=H, end_column=H + len(PREFIXES),
    )
    r += 1
    hdr(ws, r, H, "Nhóm hậu tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for suf in SUFFIXES:
        hdr(ws, r, H, f".{suf}", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, new_bests_by_cross("suffix", suf, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    # Total row
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(new_bests_by_cross("suffix", s, p) for s in SUFFIXES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Cross-tab: Số instance tốt hơn by suffix × prefix
    hdr(ws, r, H, "Số instance tốt hơn", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):         # pre-fill interior merge cells
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(
        start_row=r, end_row=r,
        start_column=H, end_column=H + len(PREFIXES),
    )
    r += 1
    hdr(ws, r, H, "Nhóm hậu tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for suf in SUFFIXES:
        hdr(ws, r, H, f".{suf}", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, instances_better_cross("suffix", suf, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(instances_better_cross("suffix", s, p) for s in SUFFIXES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Nhóm tiền tố summary
    hdr(ws, r, H,   "Nhóm",                        fill=HDR_FILL, font=HDR_FONT)
    hdr(ws, r, H+1, "Tổng số instance\n(data_file)", fill=HDR_FILL, font=HDR_FONT)
    hdr(ws, r, H+2, "Số instance\ntốt hơn",         fill=HDR_FILL, font=HDR_FONT)
    r += 1
    for p in PREFIXES:
        total   = sum(1 for (n, df) in data if n == p)
        better  = sum(1 for (n, df), d in data.items() if n == p and d["better"])
        hdr(ws, r, H,   f"{p}.", fill=GRP_FILL, font=GRP_FONT)
        hdr(ws, r, H+1, total,   fill=GRP_FILL, font=GRP_FONT)
        hdr(ws, r, H+2, better,  fill=GRP_FILL, font=GRP_FONT)
        r += 1

    # column widths for right section
    for col in range(H, H + len(PREFIXES) + 3):
        ws.column_dimensions[get_column_letter(col)].width = 14


# ── detail sheet ──────────────────────────────────────────────────────────────

def build_detail(ws, data):
    # Set all properties BEFORE merging to avoid white-background overwrite.
    hdr(ws, 1, 1, "data_file")
    hdr(ws, 2, 1, "")              # A2 — interior of A1:A2 merge
    hdr(ws, 1, 2, "run")
    hdr(ws, 2, 2, "")              # B2 — interior of B1:B2 merge
    hdr(ws, 1, 3, "Tuần tự")
    hdr(ws, 1, 4, "")              # D1 — interior of C1:D1 merge
    hdr(ws, 1, 5, "Song song")
    hdr(ws, 1, 6, "")              # F1 — interior of E1:F1 merge
    ws.merge_cells("A1:A2")
    ws.merge_cells("B1:B2")
    ws.merge_cells("C1:D1")
    ws.merge_cells("E1:F1")

    for col, label in [(3, "result"), (4, "timing_total_sec"),
                       (5, "result"), (6, "timing_total_sec")]:
        hdr(ws, 2, col, label, fill=SUB_FILL, font=SUB_FONT)

    ws.column_dimensions["A"].width = 12
    ws.column_dimensions["B"].width = 6
    for col in ["C", "D", "E", "F"]:
        ws.column_dimensions[col].width = 16

    cur_row = 3
    for n in PREFIXES:
        for mid in MIDDLES:
            for suf in SUFFIXES:
                df  = f"{n}.{mid}.{suf}"
                key = (n, df)
                if key not in data:
                    continue
                d = data[key]
                seq_runs = d["seq_runs"]
                par_runs = d["par_runs"]
                seq_best_r = d["seq_result"]
                n_runs = len(seq_runs)

                val(ws, cur_row, 1, df, font=FILE_FONT, align=LEFT)
                for i in range(n_runs):
                    sr = seq_runs[i]
                    pr = par_runs[i] if i < len(par_runs) else None
                    row = cur_row + i
                    val(ws, row, 2, sr["run"])
                    val(ws, row, 3, round(sr["result"], 6))
                    val(ws, row, 4, round(sr["time"],   3))
                    if pr:
                        is_better = pr["result"] < seq_best_r
                        pf = PatternFill("solid", fgColor="FFE2EFDA") if is_better else None
                        c_res = val(ws, row, 5, round(pr["result"], 6))
                        c_tim = val(ws, row, 6, round(pr["time"],   3))
                        if is_better:
                            c_res.fill = pf
                            c_tim.fill = pf
                    else:
                        for col in [5, 6]:
                            ws.cell(row=row, column=col).border = BORDER
                    # ensure all 6 cols always have border
                    for col in range(1, 7):
                        if ws.cell(row=row, column=col).border.left.style is None:
                            ws.cell(row=row, column=col).border = BORDER

                if n_runs > 1:
                    ws.merge_cells(
                        start_row=cur_row, end_row=cur_row + n_runs - 1,
                        start_column=1, end_column=1,
                    )

                cur_row += n_runs


# ── summary-2 sheet (baseline = paper) ───────────────────────────────────────

def build_summary2(ws, data, baseline):
    # Columns: A=data_file | B=Baseline result | C-D=Song song | E=Số new best
    hdr(ws, 1, 1, "data_file")
    hdr(ws, 2, 1, "")
    hdr(ws, 1, 2, "Baseline\n(bài báo)")
    hdr(ws, 2, 2, "")
    hdr(ws, 1, 3, "Song song")
    hdr(ws, 1, 4, "")              # interior of C1:D1
    hdr(ws, 1, 5, "Số new best\nsolution")
    hdr(ws, 2, 5, "")
    ws.merge_cells("A1:A2")
    ws.merge_cells("B1:B2")
    ws.merge_cells("C1:D1")
    ws.merge_cells("E1:E2")
    hdr(ws, 2, 3, "result",            fill=SUB_FILL, font=SUB_FONT)
    hdr(ws, 2, 4, "timing_total_sec",  fill=SUB_FILL, font=SUB_FONT)

    ws.row_dimensions[1].height = 28
    ws.row_dimensions[2].height = 28
    ws.column_dimensions["A"].width = 12
    ws.column_dimensions["B"].width = 14
    ws.column_dimensions["C"].width = 13
    ws.column_dimensions["D"].width = 13
    ws.column_dimensions["E"].width = 11

    cur_row = 3
    for n in PREFIXES:
        for mid in MIDDLES:
            for suf in SUFFIXES:
                df  = f"{n}.{mid}.{suf}"
                key = (n, df)
                if key not in data:
                    continue
                bl = baseline.get(df)
                if bl is None:
                    continue
                p_runs = data[key]["par_runs"]
                better2 = sorted(
                    [r for r in p_runs if r["result"] < bl],
                    key=lambda r: r["result"],
                )
                if not better2:
                    continue

                n_better = len(better2)
                val(ws, cur_row, 1, df,                    font=FILE_FONT, align=LEFT)
                val(ws, cur_row, 2, round(bl, 6))
                val(ws, cur_row, 3, round(better2[0]["result"], 6))
                val(ws, cur_row, 4, round(better2[0]["time"],   3))
                val(ws, cur_row, 5, n_better)

                for i, b in enumerate(better2[1:], 1):
                    val(ws, cur_row + i, 3, round(b["result"], 6))
                    val(ws, cur_row + i, 4, round(b["time"],   3))
                    for col in range(1, 6):
                        ws.cell(row=cur_row + i, column=col).border = BORDER

                if n_better > 1:
                    for col in [1, 2, 5]:
                        ws.merge_cells(
                            start_row=cur_row, end_row=cur_row + n_better - 1,
                            start_column=col, end_column=col,
                        )
                cur_row += n_better

    # ── right-side statistics ────────────────────────────────────────────────
    def getattr_df(df, attr):
        parts = df.split(".")
        if attr == "suffix": return int(parts[2])
        if attr == "middle": return int(parts[1])
        if attr == "prefix": return int(parts[0])

    def new_bests_cross(attr, val_, prefix):
        return sum(
            sum(1 for r in d["par_runs"]
                if r["result"] < baseline.get(df, float("inf")))
            for (n, df), d in data.items()
            if n == prefix and getattr_df(df, attr) == val_
        )

    def instances_better_cross(attr, val_, prefix):
        return sum(
            1 for (n, df), d in data.items()
            if n == prefix and getattr_df(df, attr) == val_
            and any(r["result"] < baseline.get(df, float("inf")) for r in d["par_runs"])
        )

    H = 7  # start column for stats (one column less than summary since no seq_time)

    r = 1
    # Cross-tab: Số new best solution by middle × prefix
    hdr(ws, r, H, "Số new best solution", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(start_row=r, end_row=r, start_column=H, end_column=H + len(PREFIXES))
    r += 1
    hdr(ws, r, H, "Nhóm trung tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for mid in MIDDLES:
        hdr(ws, r, H, f".{mid}.", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, new_bests_cross("middle", mid, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(new_bests_cross("middle", m, p) for m in MIDDLES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Cross-tab: Số instance tốt hơn by middle × prefix
    hdr(ws, r, H, "Số instance tốt hơn", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(start_row=r, end_row=r, start_column=H, end_column=H + len(PREFIXES))
    r += 1
    hdr(ws, r, H, "Nhóm trung tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for mid in MIDDLES:
        hdr(ws, r, H, f".{mid}.", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, instances_better_cross("middle", mid, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(instances_better_cross("middle", m, p) for m in MIDDLES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Cross-tab: Số new best solution by suffix × prefix
    hdr(ws, r, H, "Số new best solution", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(start_row=r, end_row=r, start_column=H, end_column=H + len(PREFIXES))
    r += 1
    hdr(ws, r, H, "Nhóm hậu tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for suf in SUFFIXES:
        hdr(ws, r, H, f".{suf}", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, new_bests_cross("suffix", suf, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(new_bests_cross("suffix", s, p) for s in SUFFIXES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Cross-tab: Số instance tốt hơn by suffix × prefix
    hdr(ws, r, H, "Số instance tốt hơn", fill=HDR_FILL, font=HDR_FONT)
    for i in range(1, len(PREFIXES) + 1):
        hdr(ws, r, H + i, "", fill=HDR_FILL, font=HDR_FONT)
    ws.merge_cells(start_row=r, end_row=r, start_column=H, end_column=H + len(PREFIXES))
    r += 1
    hdr(ws, r, H, "Nhóm hậu tố \\ Tiền tố", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i, str(p), fill=SUB_FILL, font=SUB_FONT)
    r += 1
    for suf in SUFFIXES:
        hdr(ws, r, H, f".{suf}", fill=GRP_FILL, font=GRP_FONT)
        for i, p in enumerate(PREFIXES):
            hdr(ws, r, H + 1 + i, instances_better_cross("suffix", suf, p),
                fill=GRP_FILL, font=GRP_FONT)
        r += 1
    hdr(ws, r, H, "Tổng", fill=SUB_FILL, font=SUB_FONT)
    for i, p in enumerate(PREFIXES):
        hdr(ws, r, H + 1 + i,
            sum(instances_better_cross("suffix", s, p) for s in SUFFIXES),
            fill=SUB_FILL, font=SUB_FONT)
    r += 2

    # Nhóm tiền tố summary
    hdr(ws, r, H,   "Nhóm",                         fill=HDR_FILL, font=HDR_FONT)
    hdr(ws, r, H+1, "Tổng số instance\n(data_file)", fill=HDR_FILL, font=HDR_FONT)
    hdr(ws, r, H+2, "Số instance\ntốt hơn",          fill=HDR_FILL, font=HDR_FONT)
    r += 1
    for p in PREFIXES:
        total  = sum(1 for (n, df) in data if n == p)
        better = sum(
            1 for (n, df), d in data.items()
            if n == p and any(r2["result"] < baseline.get(df, float("inf"))
                              for r2 in d["par_runs"])
        )
        hdr(ws, r, H,   f"{p}.", fill=GRP_FILL, font=GRP_FONT)
        hdr(ws, r, H+1, total,   fill=GRP_FILL, font=GRP_FONT)
        hdr(ws, r, H+2, better,  fill=GRP_FILL, font=GRP_FONT)
        r += 1

    for col in range(H, H + len(PREFIXES) + 3):
        ws.column_dimensions[get_column_letter(col)].width = 14


# ── detail-2 sheet (baseline = paper) ────────────────────────────────────────

def build_detail2(ws, data, baseline):
    # Columns: A=data_file | B=run | C=Baseline result | D-E=Song song
    hdr(ws, 1, 1, "data_file")
    hdr(ws, 2, 1, "")
    hdr(ws, 1, 2, "run")
    hdr(ws, 2, 2, "")
    hdr(ws, 1, 3, "Baseline\n(bài báo)")
    hdr(ws, 2, 3, "")
    hdr(ws, 1, 4, "Song song")
    hdr(ws, 1, 5, "")              # interior of D1:E1
    ws.merge_cells("A1:A2")
    ws.merge_cells("B1:B2")
    ws.merge_cells("C1:C2")
    ws.merge_cells("D1:E1")
    hdr(ws, 2, 4, "result",           fill=SUB_FILL, font=SUB_FONT)
    hdr(ws, 2, 5, "timing_total_sec", fill=SUB_FILL, font=SUB_FONT)

    ws.column_dimensions["A"].width = 12
    ws.column_dimensions["B"].width = 6
    ws.column_dimensions["C"].width = 14
    for col in ["D", "E"]:
        ws.column_dimensions[col].width = 16

    GREEN = PatternFill("solid", fgColor="FFE2EFDA")
    cur_row = 3
    for n in PREFIXES:
        for mid in MIDDLES:
            for suf in SUFFIXES:
                df  = f"{n}.{mid}.{suf}"
                key = (n, df)
                if key not in data:
                    continue
                d      = data[key]
                bl     = baseline.get(df)
                if bl is None:
                    continue
                par_runs = d["par_runs"]
                seq_runs = d["seq_runs"]
                n_runs   = len(seq_runs)

                val(ws, cur_row, 1, df, font=FILE_FONT, align=LEFT)
                for i in range(n_runs):
                    row = cur_row + i
                    sr  = seq_runs[i]
                    pr  = par_runs[i] if i < len(par_runs) else None
                    val(ws, row, 2, sr["run"])
                    val(ws, row, 3, round(bl, 6))
                    if pr:
                        is_better = pr["result"] < bl
                        c_res = val(ws, row, 4, round(pr["result"], 6))
                        c_tim = val(ws, row, 5, round(pr["time"],   3))
                        if is_better:
                            c_res.fill = GREEN
                            c_tim.fill = GREEN
                    else:
                        for col in [4, 5]:
                            ws.cell(row=row, column=col).border = BORDER
                    for col in range(1, 6):
                        if ws.cell(row=row, column=col).border.left.style is None:
                            ws.cell(row=row, column=col).border = BORDER

                if n_runs > 1:
                    ws.merge_cells(
                        start_row=cur_row, end_row=cur_row + n_runs - 1,
                        start_column=1, end_column=1,
                    )
                    ws.merge_cells(
                        start_row=cur_row, end_row=cur_row + n_runs - 1,
                        start_column=3, end_column=3,
                    )
                cur_row += n_runs


# ── main ─────────────────────────────────────────────────────────────────────

def build_for_strategy(strategy, baseline):
    data = load_all(strategy)
    if not data:
        print(f"  No data for strategy={strategy}, skipping.")
        return

    wb = openpyxl.Workbook()
    ws_sum  = wb.active
    ws_sum.title = "summary"
    ws_det  = wb.create_sheet("detail")
    ws_sum2 = wb.create_sheet("summary-2")
    ws_det2 = wb.create_sheet("detail-2")

    build_summary(ws_sum, data)
    build_detail(ws_det, data)
    build_summary2(ws_sum2, data, baseline)
    build_detail2(ws_det2, data, baseline)

    out = os.path.join(SCRIPT_DIR, f"best-solution-{strategy}.xlsx")
    wb.save(out)
    print(f"Saved: {out}")

    print(f"  --- vs sequential C/C++ ---")
    for n in PREFIXES:
        total   = sum(1 for (nn, df) in data if nn == n)
        better  = sum(1 for (nn, df), d in data.items() if nn == n and d["better"])
        n_bests = sum(len(d["better"]) for (nn, df), d in data.items() if nn == n)
        print(f"    {n} customers: {better}/{total} instances improved, {n_bests} new bests total")

    print(f"  --- vs paper baseline ---")
    for n in PREFIXES:
        total   = sum(1 for (nn, df) in data if nn == n)
        better  = sum(
            1 for (nn, df), d in data.items()
            if nn == n and any(r["result"] < baseline.get(df, float("inf"))
                               for r in d["par_runs"])
        )
        n_bests = sum(
            sum(1 for r in d["par_runs"]
                if r["result"] < baseline.get(df, float("inf")))
            for (nn, df), d in data.items() if nn == n
        )
        print(f"    {n} customers: {better}/{total} instances improved, {n_bests} new bests total")


def main():
    baseline   = load_baseline()
    strategies = detect_strategies()
    print(f"Detected strategies: {strategies}")
    for strategy in strategies:
        print(f"\n=== strategy={strategy} ===")
        build_for_strategy(strategy, baseline)


if __name__ == "__main__":
    main()
