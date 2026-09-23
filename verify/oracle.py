#!/isaac-sim/python.sh
"""verify/oracle.py — turn one case's run into a verdict.

cv-infra runs this right after `verify/sim.py`, in the same image, with the SAME
argv and no GPU. The last line of stdout that parses as a flat JSON object is the
case's verdict, and the platform reads TYPES, not names:

    bool   a check   — the case passes when every bool is true
    number a metric  — compared against the baseline across commits, never gates
    null   unknown   — not a failure, just excluded from the ratio
    str    a note

The question here is "did the robot get to the goal without touching anything?", so
the two bools are `reached_goal` and `collision_free`.

POLARITY (do not "simplify" this to a `collided` key): the platform passes a case when
EVERY bool is true, so a check has to be named for the GOOD outcome. A key named
`collided` would turn the gate inside out — green exactly when the robot crashed. The
collision itself is in the note, which names the first contact.

A non-zero exit means the case ERRORed (no verdict), which is different from a robot
that failed: a missing `run.json`/`contacts.json` means the sim did not get far enough
to have an opinion. A missing `trajectory.csv` only costs the two trajectory metrics.

stdlib only, so it also runs on a plain laptop python3.
"""

import argparse
import csv
import json
import math
import os
import sys

OUT_DIR = os.path.join("verify", "out")
RUN_JSON = os.path.join(OUT_DIR, "run.json")
CONTACTS_JSON = os.path.join(OUT_DIR, "contacts.json")
TRAJECTORY = os.path.join(OUT_DIR, "trajectory.csv")


def parse_args() -> argparse.Namespace:
    """Same nine axes as the sim, tolerantly parsed (the platform replays the whole argv).

    The verdict is read out of `run.json`, which records the axes the sim actually ran;
    these flags exist so a drifted argv contract fails here too, and so the two can be
    cross-checked (a stale `verify/out/` from an earlier local run is the usual cause).
    """
    p = argparse.ArgumentParser(description="verdict for one carter go-to-goal case")
    p.add_argument("--sim_time_max", type=float, required=True)
    p.add_argument("--spawn_x", type=float, required=True)
    p.add_argument("--spawn_y", type=float, required=True)
    p.add_argument("--spawn_yaw", type=float, required=True)
    p.add_argument("--goal_x", type=float, required=True)
    p.add_argument("--goal_y", type=float, required=True)
    p.add_argument("--obstacle_count", type=int, required=True)
    p.add_argument("--obstacle_kind", required=True)
    p.add_argument("--obstacle_scale", type=float, required=True)
    args, _unknown = p.parse_known_args()
    return args


def read_json(path: str) -> dict:
    with open(path) as handle:
        return json.load(handle)


def trajectory_metrics(path: str):
    """(path length [m], smallest clearance [m]) — either may be None.

    `min_clearance` is empty in every row of a case with no obstacles, which is an
    honest `null` (unknown), not a zero.
    """
    try:
        with open(path, newline="") as handle:
            rows = list(csv.DictReader(handle))
    except OSError:
        return None, None
    if not rows:
        return None, None
    length = 0.0
    # strict=False on purpose: the offset pairing is the point, so the two are never
    # the same length.
    for previous, current in zip(rows, rows[1:], strict=False):
        length += math.hypot(
            float(current["x"]) - float(previous["x"]),
            float(current["y"]) - float(previous["y"]),
        )
    clearances = [float(r["min_clearance"]) for r in rows if r.get("min_clearance")]
    return length, (min(clearances) if clearances else None)


def axes_mismatch(args: argparse.Namespace, axes: dict) -> list:
    """Axis names where argv and the recorded run disagree (empty = they match)."""
    bad = []
    for key, value in vars(args).items():
        recorded = axes.get(key)
        if isinstance(value, float) and isinstance(recorded, (int, float)):
            if abs(float(recorded) - value) > 1e-9:
                bad.append(key)
        elif recorded != value:
            bad.append(key)
    return sorted(bad)


def describe(run: dict, contacts: dict) -> str:
    """The note: what touched what, and what was standing in the lane."""
    events = contacts.get("events") or []
    if events:
        first = events[0]
        touch = (
            f"first contact {first.get('body')} <-> {first.get('kind')} "
            f"#{first.get('obstacle')} at t={float(first.get('t', 0.0)):.2f}s"
        )
    elif contacts.get("first_contact_t") is not None:
        touch = f"first contact at t={float(contacts['first_contact_t']):.2f}s (body unrecorded)"
    else:
        touch = "no obstacle contact"
    obstacles = run.get("obstacles") or []
    listing = ", ".join(
        f"{ob['kind']} x{ob['scale']} @({ob['x']:.2f}, {ob['y']:.2f})" for ob in obstacles
    )
    return f"{touch}; obstacles: {listing or 'none'}"


def main() -> int:
    args = parse_args()
    try:
        run = read_json(RUN_JSON)
        contacts = read_json(CONTACTS_JSON)
    except (OSError, ValueError) as exc:
        # The sim never got far enough to have an opinion -> ERROR lane, not a red robot.
        print(f"ERROR cannot read the case output: {exc}", file=sys.stderr, flush=True)
        return 1

    path_len, clearance = trajectory_metrics(TRAJECTORY)
    collided = bool(contacts.get("event_count")) or contacts.get("first_contact_t") is not None
    note = describe(run, contacts)
    drifted = axes_mismatch(args, run.get("axes") or {})
    if drifted:
        note += f"; WARN argv and run.json disagree on {drifted} (stale {OUT_DIR}/?)"

    verdict = {
        "reached_goal": bool(run.get("reached")),
        "collision_free": not collided,  # see POLARITY in the module docstring
        "time_to_goal_s": run.get("time_to_goal_s"),
        "final_dist_m": round(float(run.get("final_dist_m", 0.0)), 4),
        "min_clearance_m": None if clearance is None else round(clearance, 4),
        "path_len_m": None if path_len is None else round(path_len, 4),
        "note": note,
    }
    print(json.dumps(verdict))
    return 0


if __name__ == "__main__":
    sys.exit(main())
