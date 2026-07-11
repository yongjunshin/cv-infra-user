"""Custom oracle example: max_time_to_goal (scenario-adjacent plugin, D-1 (a) 2026-07-11).

This module lives NEXT TO the scenario YAML (scenarios/) and is referenced from
``acceptance_criteria`` as ``oracle: "max_time_to_goal:MaxTimeToGoalOracle"`` —
no install, no derived image, platform untouched: the platform puts this
directory on sys.path while admitting the request, and ro-mounts the same
directory into the runner container at the same absolute path for evaluation.

Rules this example follows (copy them into your own oracle):

* Module scope imports ONLY ``cv_infra.*`` (installed in the runner image) +
  stdlib — NEVER ``omni.*`` / ``isaacsim.*``: the runner composes the
  evaluation engine BEFORE the simulator boots, so a module-scope Isaac import
  crashes pre-boot (measured platform behavior; see README).
* Deterministic pure Python: read only the merged criteria view + the GT
  telemetry record (no clock, no randomness, no network).
* Defensive ``validate_params``: bad params are rejected with a clear message,
  never silently defaulted.
"""

from __future__ import annotations

from cv_infra.oracles.base import OracleBase
from cv_infra.oracles.reached_goal import DEFAULT_POS_TOL_M
from cv_infra.runner.evaluate import OracleOutcome, read_field
from cv_infra.runner.telemetry import time_to_goal_s

PARAM = "max_time_to_goal_s"


class MaxTimeToGoalOracle(OracleBase):
    """Passes iff the robot reaches the goal within ``max_time_to_goal_s`` sim-time.

    Tighter than the mission ``timeout_s``: a run that merely beats the mission
    budget can still FAIL this bound. Goal-reach is the SAME definition the MVP
    ``reached_goal`` oracle uses (first sample within ``position_tolerance_m``
    of the goal) via the platform's ``time_to_goal_s`` — one reach definition,
    not a re-implementation.
    """

    name = "max_time_to_goal"
    version = "0.1.0"

    def validate_params(self, criteria: object) -> None:
        bound = read_field(criteria, PARAM)
        if bound is None:
            raise ValueError(f"max_time_to_goal criteria require {PARAM} (seconds, > 0)")
        if isinstance(bound, bool) or not isinstance(bound, (int, float)) or bound <= 0:
            raise ValueError(f"{PARAM} must be a positive number of seconds, got {bound!r}")

    def evaluate(self, telemetry: object, criteria: object) -> OracleOutcome:
        try:
            self.validate_params(criteria)
        except ValueError as exc:
            return OracleOutcome(self.name, passed=False, reason="bad_criteria", detail=str(exc))
        bound = float(read_field(criteria, PARAM))

        goal = read_field(criteria, "goal_position")
        if goal is None:
            return OracleOutcome(
                self.name, passed=False, reason="bad_criteria", detail="missing goal_position"
            )
        samples = telemetry.gt_pose_samples
        if not samples:
            return OracleOutcome(
                self.name, passed=False, reason="no_telemetry", detail="no GT pose samples"
            )

        pos_tol = float(read_field(criteria, "position_tolerance_m", DEFAULT_POS_TOL_M))
        goal_xyz = (float(goal[0]), float(goal[1]), float(goal[2]))
        ttg = time_to_goal_s(samples, goal_xyz, pos_tol)
        if ttg is None:
            return OracleOutcome(
                self.name,
                passed=False,
                reason="not_reached",
                detail="goal never reached within tolerance — no time-to-goal to bound",
            )
        if ttg > bound:
            return OracleOutcome(
                self.name, passed=False, detail=f"time_to_goal {ttg:.2f}s exceeds bound {bound}s"
            )
        return OracleOutcome(self.name, passed=True, detail=f"time_to_goal {ttg:.2f}s <= {bound}s")
