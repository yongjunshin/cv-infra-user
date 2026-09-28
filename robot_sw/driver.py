"""robot_sw/driver.py — this robot's ENTIRE driving capability. The software under test.

What it knows: its own map of the warehouse (robot_sw/maps/, the static occupancy map it
ships with) and, every control tick, its own pose and one 2D lidar scan. What it is given:
a goal, once.

    driver = Driver(goal_x, goal_y)
    v, w = driver.step(x, y, yaw, scan)     # map-frame pose + scan in -> (m/s, rad/s) out

    scan = {"stamp": t, "angle_min": a0, "angle_increment": da, "range_max": r,
            "ranges": [...]}                 # robot frame, like sensor_msgs/LaserScan

What it does — very simple, on purpose:
  1. If the corridor toward the goal (its own width plus a margin, LOOKAHEAD_M ahead) is
     clear in both its map and its latest scan, it drives straight at the goal.
  2. Otherwise it takes the clear heading closest to the goal (up to 150 degrees off it),
     and keeps to the same side until the goal is clear again — switching sides only
     when its side has nothing clear left.
  3. It never turns in place with something within SWEEP_M: its chassis reaches 0.59 m
     behind the wheel axle, so the tail would swing into it. Then it first rolls straight
     out of the tight spot (forward if clear, else backward), and turns afterwards.
  4. It stops on arrival, and stops (stuck) when nothing at all is clear.
It has no planner, remembers nothing it saw, and does not know where the next gap is — it
keeps the nearest clear heading to the goal. That is the capability being verified, as is.

stdlib + Pillow (for the map image, which the robot's runtime bundles).
"""

import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_YAML = os.path.join(HERE, "maps", "carter_warehouse_navigation.yaml")

CRUISE_MS = 0.4  # [m/s] in the clear
CAREFUL_MS = 0.25  # [m/s] when something is within CAREFUL_M ahead
CAREFUL_M = 1.0
TURN_RATE_MAX = 1.0  # [rad/s]
HEADING_GAIN = 2.0  # rad/s of turn per rad of heading error
TURN_IN_PLACE_RAD = 0.35  # heading error above this: stop and turn first
ARRIVED_M = 0.25
HALF_WIDTH_M = 0.40  # corridor half-width: chassis half-width 0.24 m + 0.16 m margin
LOOKAHEAD_M = 1.5  # how far ahead a heading must be clear
BEHIND_M = 0.3  # ...and how far behind the axle
CANDIDATE_STEP_RAD = math.radians(5.0)
CANDIDATE_MAX_RAD = math.radians(150.0)
# Turning in place sweeps the chassis tail through a circle of ~0.64 m around the axle
# (0.59 m behind it, 0.24 m to each side); nothing may be inside that circle to turn.
SWEEP_M = 0.70
UNWIND_MS = 0.15  # [m/s] rolling straight out of a tight spot before turning
UNWIND_CLEAR_M = 0.5  # how far the straight roll-out path must be clear


def wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def load_occupied_cells(yaml_path: str) -> list:
    """World-frame centres of the occupied cells of a nav2 map (trinary, negate 0)."""
    from PIL import Image  # noqa: PLC0415 — bundled with the robot's runtime

    meta = {}
    with open(yaml_path) as handle:
        for line in handle:
            if ":" in line:
                key, value = line.split(":", 1)
                meta[key.strip()] = value.strip()
    resolution = float(meta["resolution"])
    origin_x, origin_y = (float(v) for v in meta["origin"].strip("[]").split(",")[:2])
    occupied_thresh = float(meta["occupied_thresh"])
    image = Image.open(os.path.join(os.path.dirname(yaml_path), meta["image"])).convert("L")
    width, height = image.size
    pixels = image.load()
    cells = []
    for row in range(height):
        for col in range(width):
            if (255 - pixels[col, row]) / 255.0 > occupied_thresh:
                cells.append((origin_x + (col + 0.5) * resolution, origin_y + (height - row - 0.5) * resolution))
    return cells


class Driver:
    """Go to the goal; take the nearest clear heading when the way is blocked."""

    def __init__(self, goal_x: float, goal_y: float) -> None:
        self.goal = (float(goal_x), float(goal_y))
        self.map_cells = load_occupied_cells(MAP_YAML)
        self.side = 0.0  # +1 left / -1 right while going around something, 0 otherwise
        self.heading = None  # world heading chosen at the last scan
        self.careful = False
        self.crowded = False  # something inside the tail's swing circle
        self.ahead_clear = self.behind_clear = False
        self.stamp = None

    def _obstacles_near(self, x: float, y: float, yaw: float, scan: dict) -> list:
        """Points (world frame) within reach: the latest scan's returns + the map's walls."""
        reach = LOOKAHEAD_M + HALF_WIDTH_M
        points = []
        angle = scan["angle_min"]
        for rng in scan["ranges"]:
            if rng is not None and rng < min(scan["range_max"], reach + 0.5):
                points.append((x + rng * math.cos(yaw + angle), y + rng * math.sin(yaw + angle)))
            angle += scan["angle_increment"]
        for cx, cy in self.map_cells:
            if abs(cx - x) < reach and abs(cy - y) < reach:
                points.append((cx, cy))
        return points

    @staticmethod
    def _clear(x: float, y: float, heading: float, points: list, length: float) -> float:
        """How far [m] the corridor along `heading` is clear (capped at `length`)."""
        ux, uy = math.cos(heading), math.sin(heading)
        free = length
        for px, py in points:
            along = (px - x) * ux + (py - y) * uy
            if -BEHIND_M < along < free and abs((px - x) * uy - (py - y) * ux) < HALF_WIDTH_M:
                free = max(along, 0.0)
        return free

    def _decide(self, x: float, y: float, yaw: float, scan: dict) -> None:
        goal_x, goal_y = self.goal
        to_goal = math.atan2(goal_y - y, goal_x - x)
        need = min(LOOKAHEAD_M, math.hypot(goal_x - x, goal_y - y))
        points = self._obstacles_near(x, y, yaw, scan)
        if self._clear(x, y, to_goal, points, need) >= need:
            self.side, self.heading = 0.0, to_goal
        else:
            # Its own side first; the other side only when its side has nothing clear.
            orders = ((self.side,), (-self.side,)) if self.side else ((1.0, -1.0),)
            best = None
            steps = int(CANDIDATE_MAX_RAD / CANDIDATE_STEP_RAD)
            for sides in orders:
                for k in range(1, steps + 1):
                    for side in sides:
                        heading = to_goal + side * k * CANDIDATE_STEP_RAD
                        if self._clear(x, y, heading, points, LOOKAHEAD_M) >= LOOKAHEAD_M:
                            best = (side, heading)
                            break
                    if best:
                        break
                if best:
                    break
            if best:
                self.side, self.heading = best
            else:
                self.heading = None  # nothing clear anywhere: stuck
        self.careful = self._clear(x, y, yaw, points, CAREFUL_M) < CAREFUL_M
        self.crowded = any(math.hypot(px - x, py - y) < SWEEP_M for px, py in points)
        self.ahead_clear = self._clear(x, y, yaw, points, UNWIND_CLEAR_M) >= UNWIND_CLEAR_M
        # Backing up: the tail leads, so the corridor starts at the tail (0.59 m behind).
        self.behind_clear = self._clear(x, y, yaw + math.pi, points, UNWIND_CLEAR_M + 0.6) >= UNWIND_CLEAR_M + 0.6

    def step(self, x: float, y: float, yaw: float, scan: dict) -> tuple[float, float]:
        goal_x, goal_y = self.goal
        if math.hypot(goal_x - x, goal_y - y) <= ARRIVED_M:
            return 0.0, 0.0
        if scan is not None and scan.get("stamp") != self.stamp:
            self.stamp = scan.get("stamp")
            self._decide(x, y, yaw, scan)
        if self.heading is None:
            return 0.0, 0.0  # stuck: nothing clear anywhere
        error = wrap(self.heading - yaw)
        w = max(-TURN_RATE_MAX, min(TURN_RATE_MAX, HEADING_GAIN * error))
        if abs(error) <= TURN_IN_PLACE_RAD:
            return (CAREFUL_MS if self.careful else CRUISE_MS), w
        if not self.crowded:
            return 0.0, w  # turn in place: the tail's swing circle is empty
        if self.ahead_clear:
            return UNWIND_MS, 0.0  # roll straight out of the tight spot first
        if self.behind_clear:
            return -UNWIND_MS, 0.0
        return 0.0, 0.0  # boxed in: neither turning nor rolling is safe
