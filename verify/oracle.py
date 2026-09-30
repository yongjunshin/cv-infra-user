#!/isaac-sim/python.sh
"""verify/oracle.py — turn one case's run into a verdict.

cv-infra runs this right after `verify/sim.py`, in the same image, with the SAME argv and
no GPU. The last line of stdout that parses as a flat JSON object is the case's verdict,
and the platform reads TYPES, not names:

    bool   a check   — the case passes when every bool is true
    number a metric  — compared against the baseline across commits, never gates
    null   unknown   — not a failure, just excluded from the ratio
    str    a note

The question is "did the robot's software get it to the goal without running into
anything?", so the two bools are `reached_goal` and `collision_free`. "Anything" is
literal: the harness hears every contact of every robot body, and only the floor carrying
the robot is excluded — a prop, a shelf or a wall all count.

POLARITY (do not "simplify" this to a `collided` key): the platform passes a case when
EVERY bool is true, so a check has to be named for the GOOD outcome. A key named
`collided` would turn the gate inside out — green exactly when the robot crashed. The
collision itself is in the note, which names the first one.

A non-zero exit means the case ERRORed (no verdict), which is different from a robot
that failed: a missing `run.json`/`contacts.json` means the sim did not get far enough to
have an opinion. A missing `trajectory.csv` only costs the two trajectory metrics.

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
SLOTS = tuple("abcdefghijklmno")


def parse_args() -> argparse.Namespace:
    """Same axes as the sim (start, goal, slot_a..slot_o), tolerantly parsed.

    The verdict is read out of `run.json`, which records the axes the sim actually ran;
    these flags exist so a drifted argv contract fails here too, and so the two can be
    cross-checked (a stale `verify/out/` from an earlier local run is the usual cause).
    """
    p = argparse.ArgumentParser(description="verdict for one carter go-to-goal case")
    p.add_argument("--start", required=True)
    p.add_argument("--goal", required=True)
    for slot in SLOTS:
        p.add_argument(f"--slot_{slot}", required=True)
    args, _unknown = p.parse_known_args()
    return args


def read_json(path: str) -> dict:
    with open(path) as handle:
        return json.load(handle)


def trajectory_metrics(path: str):
    """(path length [m], smallest clearance [m]) — either may be None.

    `min_clearance` is empty in every row of a case with no props, which is an honest
    `null` (unknown), not a zero.
    """
    try:
        with open(path, newline="") as handle:
            rows = list(csv.DictReader(handle))
    except OSError:
        return None, None
    if not rows:
        return None, None
    length = 0.0
    # strict=False on purpose: the offset pairing is the point, so the two are never the
    # same length.
    for previous, current in zip(rows, rows[1:], strict=False):
        length += math.hypot(
            float(current["x"]) - float(previous["x"]),
            float(current["y"]) - float(previous["y"]),
        )
    clearances = [float(r["min_clearance"]) for r in rows if r.get("min_clearance")]
    return length, (min(clearances) if clearances else None)


def axes_mismatch(args: argparse.Namespace, axes: dict) -> list:
    """Axis names where argv and the recorded run disagree (empty = they match)."""
    return sorted(key for key, value in vars(args).items() if axes.get(key) != value)


def describe(run: dict, contacts: dict) -> str:
    """The note: how it ended, what it hit first, and what stood where."""
    first = contacts.get("first_collision")
    if first:
        hit = f"first collision {first.get('body')} <-> {first.get('label')} at t={float(first.get('t', 0.0)):.2f}s"
    else:
        hit = "no collision"
    props = ", ".join(f"{ob['slot']}={ob['kind']}" for ob in run.get("obstacles") or []) or "none"
    start, goal = run.get("start") or {}, run.get("goal") or {}
    sha = ((run.get("robot_sw") or {}).get("sha256") or "")[:12]
    pictures = run.get("pictures") or {}
    picture = pictures.get("result") if pictures.get("result") and os.path.exists(pictures["result"]) else None
    return (
        f"{start.get('name')}->{goal.get('name')}: ended by {run.get('end_reason')}; {hit}; "
        f"props {props}; robot_sw {sha}; picture {picture or 'none'}"
    )


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
    collided = bool(contacts.get("collision_event_count")) or contacts.get("first_collision") is not None
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
