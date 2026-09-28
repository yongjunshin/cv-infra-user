"""robot_sw/straight_driver.py — this robot's ENTIRE driving capability. The software under test.

It is given a goal once, then called at the control rate with nothing but its own pose,
and answers with a body velocity command:

    driver = StraightDriver(goal_x, goal_y)
    v, w = driver.step(x, y, yaw)        # map-frame pose in -> (m/s, rad/s) out

What it does: turn in place until it faces the goal, then drive there in a straight line
(holding the heading), and stop on arrival. What it CANNOT do: it has no sensors and no
map, so it cannot see — let alone avoid — anything on that line. When something is in the
way, it drives into it.

That is the capability being verified, as is. The harness in verify/ builds the world,
hands this module the goal and the pose, turns its (v, w) into wheel spin and watches;
it adds no intelligence of its own. Making the robot better (perception, avoidance, a
planner) is a change to THIS folder — the harness, the input space and the oracle stay.

stdlib only, no Isaac: this is robot code, and it runs wherever the robot runs.
"""

import math

SPEED_MS = 0.4  # cruise speed once aligned [m/s]
TURN_RATE_MAX = 0.8  # [rad/s]
HEADING_GAIN = 2.0  # rad/s of turn per rad of heading error — turning in place and holding the line
ALIGNED_RAD = 0.05  # |heading error| below this counts as "facing the goal": start driving
ARRIVED_M = 0.25  # stop within this distance of the goal


class StraightDriver:
    """Face the goal, then drive straight at it. No perception, no map, no memory of the world."""

    def __init__(self, goal_x: float, goal_y: float) -> None:
        self.goal = (float(goal_x), float(goal_y))
        self.driving = False  # latched: once it sets off, it does not stop to re-aim

    def step(self, x: float, y: float, yaw: float) -> tuple[float, float]:
        goal_x, goal_y = self.goal
        dx, dy = goal_x - x, goal_y - y
        if math.hypot(dx, dy) <= ARRIVED_M:
            return 0.0, 0.0
        bearing = math.atan2(dy, dx) - yaw
        error = math.atan2(math.sin(bearing), math.cos(bearing))
        w = max(-TURN_RATE_MAX, min(TURN_RATE_MAX, HEADING_GAIN * error))
        if not self.driving:
            if abs(error) > ALIGNED_RAD:
                return 0.0, w  # still turning in place
            self.driving = True
        return SPEED_MS, w
