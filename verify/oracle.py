#!/usr/bin/env python3
"""verify/oracle.py — turn one case's trajectory into a verdict.

cv-infra runs this right after `verify/sim.py`, in the same image, with the SAME
argv and no GPU. The last line of stdout that parses as a flat JSON object is the
case's verdict, and the platform reads TYPES, not names:

    bool   a check   — the case passes when every bool is true
    number a metric  — compared against the baseline across commits, never gates
    null   unknown   — not a failure, just excluded from the ratio
    str    a note

A non-zero exit means the case ERRORed (no verdict), which is different from a
robot that failed. stdlib only, so it also runs on a plain laptop python3.
"""

import argparse
import csv
import json
import math
import os
import sys

TRAJECTORY = os.path.join("verify", "out", "trajectory.csv")

# "Did it actually drive?" — an ideal straight run covers v*t metres, and a real one
# loses some of that to spin-up, to arcing when yaw_rate is non-zero, and to the
# warehouse floor. Half of the ideal is the line between "drove" and "stuck".
MIN_TRAVEL_FRACTION = 0.5
# ~20°: a wheeled base on a flat warehouse floor tips far less; more than this means
# it climbed something or rolled.
MAX_TILT_RAD = 0.35


def parse_args() -> argparse.Namespace:
    """Same axes as the sim, tolerantly parsed (the platform replays the whole argv)."""
    p = argparse.ArgumentParser(description="verdict for one carter drive case")
    p.add_argument("--drive_v", type=float, required=True)
    p.add_argument("--drive_t", type=float, required=True)
    p.add_argument("--yaw_rate", type=float, required=True)
    args, _unknown = p.parse_known_args()
    return args


def read_rows(path: str) -> list[dict]:
    with open(path, newline="") as handle:
        return list(csv.DictReader(handle))


def main() -> int:
    args = parse_args()
    try:
        rows = read_rows(TRAJECTORY)
    except OSError as exc:
        print(f"ERROR cannot read {TRAJECTORY}: {exc}", file=sys.stderr, flush=True)
        return 1
    if not rows:
        print(f"ERROR {TRAJECTORY} has no samples", file=sys.stderr, flush=True)
        return 1

    first, last = rows[0], rows[-1]
    displacement = math.hypot(
        float(last["x"]) - float(first["x"]),
        float(last["y"]) - float(first["y"]),
    )
    max_tilt = max(max(abs(float(r["roll"])), abs(float(r["pitch"]))) for r in rows)
    expected = MIN_TRAVEL_FRACTION * args.drive_v * args.drive_t

    verdict = {
        "moved": displacement >= expected,
        "upright": max_tilt < MAX_TILT_RAD,
        "displacement_m": round(displacement, 4),
        "final_z_m": round(float(last["z"]), 4),
        "note": f"{len(rows)} samples over {float(last['t']):.2f} sim-s; "
        f"expected >= {expected:.2f} m; max tilt {max_tilt:.3f} rad",
    }
    print(json.dumps(verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
