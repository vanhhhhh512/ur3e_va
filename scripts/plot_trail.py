#!/usr/bin/env python3
"""Ve lai ban ve tu file CSV: quy dao DU DINH va vet but THUC TE, trong he toa do canvas.

    python3 scripts/plot_trail.py --planned out/planned.csv --actual out/trail.csv \
        --out out/ban_ve.png

Ngoai hinh anh, script in ra sai so bam quy dao: voi moi diem thuc te, tim diem gan nhat
tren quy dao du dinh cua CUNG mot net, roi bao trung binh / lon nhat.
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import OrderedDict

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402


def read_csv(path):
    strokes = OrderedDict()
    with open(path, newline="") as handle:
        for row in csv.DictReader(handle):
            strokes.setdefault(row["stroke"], []).append((float(row["u"]), float(row["v"])))
    return strokes


def nearest_distance(point, polyline):
    u, v = point
    return min(math.hypot(u - pu, v - pv) for pu, pv in polyline)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--planned", required=True, help="CSV quy dao du dinh")
    parser.add_argument("--actual", default="", help="CSV vet but thuc te (tuy chon)")
    parser.add_argument("--out", default="ban_ve.png")
    args = parser.parse_args()

    planned = read_csv(args.planned)
    actual = read_csv(args.actual) if args.actual else {}

    figure, axes = plt.subplots(figsize=(8, 6))
    colours = {
        "hinh_tron": "#1f9fd8",
        "chu_v": "#e2571e",
        "chu_a_khung": "#54c04d",
        "chu_a_ngang": "#54c04d",
    }

    for name, points in planned.items():
        axes.plot(
            [p[0] for p in points],
            [p[1] for p in points],
            "--",
            color=colours.get(name, "#888888"),
            linewidth=1.2,
            label=f"{name} (du dinh)",
        )
    for name, points in actual.items():
        axes.plot(
            [p[0] for p in points],
            [p[1] for p in points],
            "-",
            color=colours.get(name, "#333333"),
            linewidth=2.2,
            alpha=0.85,
            label=f"{name} (thuc te)",
        )

    axes.set_aspect("equal")
    axes.grid(alpha=0.3)
    axes.set_xlabel("u (m) — ngang tren mat phang ve")
    axes.set_ylabel("v (m) — doc tren mat phang ve")
    axes.set_title("UR3e: hinh tron + chu V + chu A tren mat phang 2D")
    axes.legend(loc="upper center", bbox_to_anchor=(0.5, -0.12), ncol=4, fontsize=7)
    figure.tight_layout()
    figure.savefig(args.out, dpi=140)
    print(f"Da luu hinh: {args.out}")

    for name, points in actual.items():
        if name not in planned:
            continue
        errors = [nearest_distance(p, planned[name]) for p in points]
        print(
            f"[{name}] {len(points)} diem thuc te | sai so trung binh "
            f"{1000 * sum(errors) / len(errors):.2f} mm | lon nhat {1000 * max(errors):.2f} mm"
        )


if __name__ == "__main__":
    main()
