#!/isaac-sim/python.sh
"""verify/sim.py — one verification case: drive Nova Carter to a goal through clutter.

A STANDARD Isaac Sim standalone script. cv-infra never imports this file and knows
nothing about what it means; per PICT case it runs, inside the stock Isaac image,

    verify/sim.py --sim_time_max=40 --spawn_x=-6.0 --spawn_y=-1.0 --spawn_yaw=1.2708 \
      --goal_x=-6.0 --goal_y=3.0 --obstacle_count=0 --obstacle_kind=cardbox \
      --obstacle_scale=1.0

through a `/bin/sh -lc 'exec "$0" "$@"'` wrapper — this file is the executable
entrypoint and the shebang above, not the platform, picks the interpreter —
with this repository checked out read-only at the working directory, `verify/out/`
overlaid read-write, and `CV_SEED` in the environment. Which scene, which robot,
which obstacles, how it is driven and what gets written are OURS — the platform only
supplies the runtime, the input space and the collection of `verify/out/`.

WHAT ONE CASE IS: teleport the robot to the spawn pose, drop `obstacle_count` stock
warehouse props of one kind and scale ON the straight spawn->goal segment, then let
the ~60-line controller below — the stand-in "robot SW", no ROS and no nav2 — drive
toward the goal until it arrives or `sim_time_max` sim-seconds are spent. Recorded:
the trajectory, every PhysX contact between a robot body and a prop, and a run
summary. `verify/oracle.py` turns those three files into {reached_goal, collision_free, …}.

EXIT CODE IS NOT A VERDICT (platform gotcha): `SimulationApp.close()` ends the
process with status 0 no matter what happened, and the stock `python.sh` squashes a
non-zero status to 1. pass/fail is decided afterwards by `verify/oracle.py` reading
`verify/out/`. A non-zero exit here can therefore only mean "this case ERRORed" —
which is what the guards below are for, and why they run before boot. Everything the
oracle needs is written BEFORE `close()`.

ORDERING (hard rule): `SimulationApp(...)` is constructed BEFORE any `omni.*` /
`isaacsim.*` / `pxr.*` / `numpy` import — those modules do not exist (or are not the
image's) until the app has booted. Hence the stdlib-only imports at the top and the
in-function Isaac imports further down.

HEADLESS: a CI container has no display, so headless is the default; `--gui` is for
a developer running this at a desktop. Booting with a GUI in CI hangs or crashes.
"""

# stdlib only up here — see ORDERING above.
import argparse
import csv
import json
import math
import os
import sys

# The official ROS 2 navigation sample: warehouse + Nova Carter, all in one asset.
# Resolved against the Isaac assets root, which the image knows how to reach. We open
# it for its warehouse and its robot only — the scene's own ROS 2 OmniGraphs are left
# unsubscribed and idle (see "the drive path" in README.md).
SCENE_USD = "/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"

# The robot WRAPPER Xform — the prim we teleport. A candidate LIST, not a single path:
# NVIDIA's own prim naming has moved between asset revisions, and a list degrades to a
# loud "none of these exist" instead of a silently wrong pin. The chassis (articulation
# root + rigid body, and where we read the ground-truth pose) hangs below it.
ROBOT_CANDIDATES = (
    "/World/Nova_Carter_ROS",
    "/World/Carter_ROS",
)
CHASSIS_CHILD = "chassis_link"

# Where our obstacles live on the stage. The contact callback recognises a prop by this
# prefix, so the index must be the first segment after it.
OBSTACLE_PREFIX = "/World/cv_obs_"

# Stock Simple_Warehouse props: (asset, x size, y size) in metres at scale 1.0, all
# MEASURED on this image 2026-09-23 with a BBoxCache over a freshly referenced prim.
# min_z is 0 for all three, so translating to z = 0 stands them on the floor. Each
# carries CollisionAPI but NOT RigidBodyAPI: static colliders that do not get shoved
# aside, which is what makes "did it touch?" a clean question. Mind the asset spelling
# — "Barel" has one r and "Palette" is not "Pallet".
PROPS = {
    "cardbox": ("/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01.usd", 0.70, 0.50),
    "barrel": ("/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_A_01.usd", 0.60, 0.71),
    "pallet": ("/Isaac/Environments/Simple_Warehouse/Props/SM_PaletteA_01.usd", 1.21, 1.00),
}

# Where the props go: fractions of the spawn->goal segment and an alternating lateral
# offset off it. Fixed, so one PICT row names exactly one world — same axes in, same
# clutter out, exact replay.
OBSTACLE_FRACTIONS = (0.40, 0.65)
OBSTACLE_LATERAL_M = (0.25, -0.25)

# Wheel geometry — NOT a guess: these are the scene's OWN authored values, read off the
# DifferentialController node inside carter_warehouse_navigation.usd (MEASURED
# 2026-09-23). They convert the controller's (v, w) into wheel spin.
WHEEL_RADIUS_M = 0.14
WHEEL_TRACK_M = 0.413

# Checkout-relative, because the platform mounts the case's output directory over
# exactly this path (and `cd`s to the checkout). Same paths when run locally.
OUT_DIR = os.path.join("verify", "out")
OUT_TRAJECTORY = os.path.join(OUT_DIR, "trajectory.csv")
OUT_CONTACTS = os.path.join(OUT_DIR, "contacts.json")
OUT_RUN = os.path.join(OUT_DIR, "run.json")

WARMUP_STEPS = 8  # physics/renderer are unreliable on the very first steps
MAX_EVENTS = 200  # a 26 s brush against a box produced ~6200 events — cap the dump
EXIT_ERROR = 1
EXIT_NO_CONSENT = 3


def log(msg: str) -> None:
    print(f"[cv-carter] {msg}", flush=True)


def parse_args() -> argparse.Namespace:
    """The axes of verify/param_space.pict, one flag each (plus the local --gui).

    Strict parsing on purpose: an axis added to the PICT model without a flag here
    fails loudly on argv instead of being silently ignored.
    """
    p = argparse.ArgumentParser(description="carter go-to-goal case for cv-infra")
    p.add_argument("--sim_time_max", type=float, required=True, help="budget [sim s]")
    p.add_argument("--spawn_x", type=float, required=True, help="spawn x [m, map frame]")
    p.add_argument("--spawn_y", type=float, required=True, help="spawn y [m, map frame]")
    p.add_argument("--spawn_yaw", type=float, required=True, help="spawn heading [rad]")
    p.add_argument("--goal_x", type=float, required=True, help="goal x [m, map frame]")
    p.add_argument("--goal_y", type=float, required=True, help="goal y [m, map frame]")
    p.add_argument("--obstacle_count", type=int, required=True, help="props on the segment")
    p.add_argument("--obstacle_kind", required=True, choices=sorted(PROPS), help="prop asset")
    p.add_argument("--obstacle_scale", type=float, required=True, help="uniform USD scale")
    p.add_argument("--gui", action="store_true", help="show the viewport (desktop only)")
    return p.parse_args()


def consent_guard() -> None:
    """Refuse to boot Isaac without the operator's runtime consent.

    No acceptance literal is committed anywhere; the platform passes the operator's
    own `ACCEPT_EULA` through into the case container.
    """
    if not os.environ.get("ACCEPT_EULA"):
        print(
            "ERROR: NVIDIA Isaac Sim EULA has not been accepted for this run "
            "(env ACCEPT_EULA is empty). Boot refused.",
            file=sys.stderr,
            flush=True,
        )
        sys.exit(EXIT_NO_CONSENT)


# --------------------------------------------------------------------------------------
# The world, as a pure function of the axes. Nothing below touches Isaac, so it can be
# imported and checked on a laptop (see the offline placement check in the task notes).
# --------------------------------------------------------------------------------------


def wrap_pi(angle: float) -> float:
    """Fold an angle into (-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quat(quat_wxyz) -> float:
    """Yaw [rad] from a (w, x, y, z) quaternion — stdlib math, no numpy."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def obstacle_poses(spawn, goal, count: int, kind: str, scale: float) -> list[dict]:
    """Where the props stand for one case — spawn/goal in, world out, no state.

    `radius` is the prop's circumscribed footprint radius (half of its longer side,
    times the scale). The controller and the clearance metric both approximate a prop
    by that circle; the physics uses the real triangle mesh, so a contact can happen
    a little inside the circle for a box seen corner-on.
    """
    if count > len(OBSTACLE_FRACTIONS):
        raise ValueError(f"obstacle_count={count} exceeds the {len(OBSTACLE_FRACTIONS)} slots")
    asset, size_x, size_y = PROPS[kind]
    (spawn_x, spawn_y), (goal_x, goal_y) = spawn, goal
    heading = math.atan2(goal_y - spawn_y, goal_x - spawn_x)
    length = math.hypot(goal_x - spawn_x, goal_y - spawn_y)
    along = (math.cos(heading), math.sin(heading))  # unit vector spawn -> goal
    across = (-along[1], along[0])  # its left-hand normal
    radius = 0.5 * max(size_x, size_y) * scale
    poses = []
    for i in range(count):
        offset = OBSTACLE_FRACTIONS[i] * length
        lateral = OBSTACLE_LATERAL_M[i]
        poses.append(
            {
                "kind": kind,
                "asset": asset,
                "x": spawn_x + offset * along[0] + lateral * across[0],
                "y": spawn_y + offset * along[1] + lateral * across[1],
                # Turned with the lane, so the prop's own long axis runs ALONG the
                # segment and its short axis lies across it.
                "yaw": heading,
                "scale": scale,
                "radius": radius,
            }
        )
    return poses


def obstacle_index(prim_path: str):
    """`/World/cv_obs_1/SM_…` -> 1; any other prim -> None."""
    if not prim_path.startswith(OBSTACLE_PREFIX):
        return None
    head = prim_path[len(OBSTACLE_PREFIX) :].split("/", 1)[0]
    return int(head) if head.isdigit() else None


def min_clearance(x: float, y: float, obstacles: list[dict]):
    """Distance [m] to the nearest prop SURFACE (circle approximation), or None."""
    if not obstacles:
        return None
    return min(math.hypot(ob["x"] - x, ob["y"] - y) - ob["radius"] for ob in obstacles)


def make_contact_log(obstacles: list[dict], robot_path: str, to_sdf_path):
    """The PhysX contact callback and the record it fills — (callback, log).

    `to_sdf_path` is `PhysicsSchemaTools.intToSdfPath`, passed IN so this stays a plain
    stdlib function at module scope (see ORDERING). The caller keeps `log["t"]` current:
    the callback fires from inside `world.step()`, so that is the sim time of the step
    the contact happened in. `events` is capped, `counts`/`pairs`/`first_contact_t`
    are not — a 26 s brush against one box produced ~6200 events.
    """
    log_ = {
        "t": 0.0,
        "first_contact_t": None,
        "foreign_body": None,
        "events": [],
        "counts": {},
        "pairs": set(),
    }

    def on_contact(headers, data) -> None:
        for header in headers:
            actor0 = str(to_sdf_path(header.actor0))
            actor1 = str(to_sdf_path(header.actor1))
            index0, index1 = obstacle_index(actor0), obstacle_index(actor1)
            if (index0 is None) == (index1 is None):
                continue  # neither side is ours, or both are: not a robot-vs-prop touch
            index = index0 if index0 is not None else index1
            # The other side is whatever touched the prop. We do NOT filter it down to
            # the robot prim: if something else in the warehouse ever shows up here we
            # want it LOUD in the verdict, not silently dropped into "no contact".
            other = actor1 if index0 is not None else actor0
            body = other.rsplit("/", 1)[-1]
            if not other.startswith(robot_path) and log_["foreign_body"] is None:
                log_["foreign_body"] = other
            event_type = str(header.type).split(".")[-1]
            log_["counts"][event_type] = log_["counts"].get(event_type, 0) + 1
            log_["pairs"].add((body, obstacles[index]["kind"], index))
            if log_["first_contact_t"] is None:
                log_["first_contact_t"] = round(log_["t"], 4)
            if len(log_["events"]) < MAX_EVENTS:
                log_["events"].append(
                    {
                        "t": round(log_["t"], 4),
                        "type": event_type,
                        "body": body,
                        "obstacle": index,
                        "kind": obstacles[index]["kind"],
                    }
                )

    return on_contact, log_


# --------------------------------------------------------------------------------------
# The stand-in "robot SW". Every constant it is tuned by lives in this one block. It is
# deliberately simple — go-to-goal with a repulsive steering term off the KNOWN prop
# poses (map-based, no perception) — and it is meant to clear a single 1.0x cardbox
# while possibly failing a pair of 1.5x barrels. That spread is the point: a red case
# is this controller's failure, not the platform's.
# --------------------------------------------------------------------------------------
K_HEADING = 2.0  # rad/s of yaw command per rad of bearing error
# rad/s per unit of zone depth (it saturates against W_MAX_RADS long before the prop's
# centre). It has to out-pull K_HEADING or the goal term drags the robot straight back
# into the prop: the two balance at a heading offset of (K_AVOID / K_HEADING) * depth.
# CHOSEN by a kinematic rollout of all 15 rows (no physics, circular props): 2.5 grazed
# 8 of them, 4.5 clears 13 and still times out on the two hardest pairs.
K_AVOID = 4.5
V_MAX_MS = 0.4
V_MIN_AVOID_MS = 0.15  # never crawl to a halt in a zone — that is a timeout, not avoidance
W_MAX_RADS = 0.8
ROBOT_RADIUS_M = 0.55  # Nova Carter's footprint, generously (0.413 m track, ~0.9 m long)
INFLUENCE_M = 0.60  # margin on top of (robot + prop) radii where steering starts
HEAD_ON_RAD = 0.35  # inside this the "away" side is a coin flip — use the lane tie-break
LANE_CENTRE_X = -6.0  # MEASURED free floor is x in [-6.3, -5.7]; its centre is the roomy side
GOAL_RADIUS_M = 0.30  # "reached"


def control(x: float, y: float, yaw: float, goal, obstacles: list[dict]):
    """One control step: pose + known clutter -> (v [m/s], w [rad/s]). Deterministic."""
    goal_x, goal_y = goal
    error = wrap_pi(math.atan2(goal_y - y, goal_x - x) - yaw)
    w = K_HEADING * error
    v = V_MAX_MS * max(0.0, math.cos(error))  # do not drive forward while facing away

    for ob in obstacles:
        dx, dy = ob["x"] - x, ob["y"] - y
        distance = math.hypot(dx, dy)
        zone = ob["radius"] + ROBOT_RADIUS_M + INFLUENCE_M
        if distance >= zone:
            continue
        bearing = wrap_pi(math.atan2(dy, dx) - yaw)
        if abs(bearing) > 0.5 * math.pi:
            continue  # already abeam or behind it; swerving now only costs distance
        depth = (zone - distance) / zone  # 0 at the rim, -> 1 at the prop's centre
        away = -1.0 if bearing > 0.0 else 1.0  # turn the nose off the obstacle
        # Room towards the lane centre, expressed as a turn: > 0 iff turning LEFT moves
        # the robot towards x = LANE_CENTRE_X (left-hand unit vector is (-sin, cos)).
        room = (LANE_CENTRE_X - x) * -math.sin(yaw)
        if abs(bearing) < HEAD_ON_RAD and abs(room) > 1e-6:
            # Head-on: "away" flip-flops step to step, so break the tie with the map and
            # commit to the side that has more lane left on it.
            side = 1.0 if room > 0.0 else -1.0
        else:
            side = away
        w += side * K_AVOID * depth
        v = min(v, max(V_MIN_AVOID_MS, V_MAX_MS * (1.0 - depth)))

    return v, max(-W_MAX_RADS, min(W_MAX_RADS, w))


# --------------------------------------------------------------------------------------
# Output. Written before SimulationApp.close(), because the exit code carries nothing.
# --------------------------------------------------------------------------------------


def write_trajectory(samples: list[tuple]) -> None:
    """One row per physics step. An empty cell is JSON `null` to the oracle."""
    with open(OUT_TRAJECTORY, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            ("t", "x", "y", "yaw", "v_cmd", "w_cmd", "dist_to_goal", "min_clearance")
        )
        for row in samples:
            writer.writerow("" if value is None else f"{value:.6f}" for value in row)


def write_json(path: str, payload: dict) -> None:
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=1, sort_keys=True)
        handle.write("\n")


def run(simulation_app, args: argparse.Namespace) -> None:
    """Build the case's world, drive it, write verify/out/."""
    # Isaac imports are legal only here — after SimulationApp booted. numpy included:
    # the one we must use is the image's, which the boot puts on the path.
    import numpy as np  # noqa: PLC0415
    import omni.usd  # noqa: PLC0415
    from isaacsim.core.api import World  # noqa: PLC0415
    from isaacsim.core.prims import SingleArticulation, SingleXFormPrim  # noqa: PLC0415
    from isaacsim.core.utils.stage import is_stage_loading  # noqa: PLC0415
    from isaacsim.storage.native import get_assets_root_path  # noqa: PLC0415
    from omni.physx import get_physx_simulation_interface  # noqa: PLC0415
    from pxr import Gf, PhysicsSchemaTools, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: PLC0415

    spawn = (args.spawn_x, args.spawn_y)
    goal = (args.goal_x, args.goal_y)
    obstacles = obstacle_poses(
        spawn, goal, args.obstacle_count, args.obstacle_kind, args.obstacle_scale
    )
    for i, ob in enumerate(obstacles):
        log(
            f"obstacle {i}: {ob['kind']} x{ob['scale']} at ({ob['x']:.3f}, {ob['y']:.3f}) "
            f"yaw {ob['yaw']:.3f} rad, footprint radius {ob['radius']:.3f} m"
        )
    if len(obstacles) == 2:
        gap = math.hypot(
            obstacles[0]["x"] - obstacles[1]["x"], obstacles[0]["y"] - obstacles[1]["y"]
        ) - (obstacles[0]["radius"] + obstacles[1]["radius"])
        if gap <= 0.0:
            # Not fatal — two static colliders may interpenetrate — but it turns the case
            # into a blocked corridor instead of clutter, so it has to be visible in the
            # log. COMPUTED over the whole input space: only a 1.0x pallet pair on the
            # shortest (4 m) segment lands here, and no row of the current PICT array does.
            log(f"WARN the two props overlap by {-gap:.3f} m — this case is a wall, not clutter")

    assets_root = get_assets_root_path()
    if not assets_root:
        raise RuntimeError("Isaac assets root could not be resolved (no local/remote assets)")
    scene_url = assets_root + SCENE_USD

    if not omni.usd.get_context().open_stage(scene_url):
        raise RuntimeError(f"open_stage failed for {scene_url!r}")
    while is_stage_loading():
        simulation_app.update()
    log(f"scene loaded: {scene_url}")

    world = World(stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()

    robot_path = next((p for p in ROBOT_CANDIDATES if stage.GetPrimAtPath(p).IsValid()), None)
    if robot_path is None:
        roots = [str(p.GetPath()) for p in stage.GetPseudoRoot().GetAllChildren()]
        raise RuntimeError(
            f"robot prim not found — tried {list(ROBOT_CANDIDATES)}; stage roots: {roots} "
            "(sample asset naming changed?)"
        )
    chassis_path = f"{robot_path}/{CHASSIS_CHILD}"

    # TELEPORT (MEASURED 2026-09-23): the WRAPPER Xform, and BEFORE world.reset(). Moving
    # the articulation root (the chassis) itself before play breaks the robot; the pose
    # after reset came back exactly as requested.
    SingleXFormPrim(robot_path).set_world_pose(
        np.array([spawn[0], spawn[1], 0.0]),
        np.array([math.cos(0.5 * args.spawn_yaw), 0.0, 0.0, math.sin(0.5 * args.spawn_yaw)]),
    )

    # OBSTACLES, also BEFORE world.reset(): the props' triangle-mesh colliders are baked
    # at scene init, and the collider scales with the prim (MEASURED: a 1.5x cardbox stops
    # the robot 0.175 m earlier — exactly its extra half-width). Op order is T * Rz * S.
    for i, ob in enumerate(obstacles):
        prim = stage.DefinePrim(f"{OBSTACLE_PREFIX}{i}", "Xform")
        prim.GetReferences().AddReference(assets_root + ob["asset"])
        xform = UsdGeom.Xformable(prim)
        xform.AddTranslateOp().Set(Gf.Vec3d(ob["x"], ob["y"], 0.0))
        xform.AddRotateZOp().Set(math.degrees(ob["yaw"]))
        xform.AddScaleOp().Set(Gf.Vec3f(ob["scale"], ob["scale"], ob["scale"]))
        # The only contact readout that works against STATIC props AND names which body
        # hit what: the raw PhysX contact report, applied per collider prim. (A
        # ContactSensor on the chassis MISSED a box the wheels hit first, and the tensor
        # API's contact filters do not see static colliders at all — both MEASURED.)
        colliders = 0
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.CollisionAPI):
                PhysxSchema.PhysxContactReportAPI.Apply(child).CreateThresholdAttr().Set(0.0)
                colliders += 1
        if colliders == 0:
            raise RuntimeError(f"{ob['asset']!r} referenced no collider — cannot detect contact")

    on_contact, contacts = make_contact_log(
        obstacles, robot_path, PhysicsSchemaTools.intToSdfPath
    )
    subscription = get_physx_simulation_interface().subscribe_contact_report_events(on_contact)

    world.reset()
    for _ in range(WARMUP_STEPS):
        world.step(render=True)

    chassis = SingleXFormPrim(chassis_path)
    robot = SingleArticulation(robot_path)
    robot.initialize()
    names = list(robot.dof_names)
    left = [i for i, n in enumerate(names) if "wheel_left" in n]
    right = [i for i, n in enumerate(names) if "wheel_right" in n]
    if not left or not right:
        raise RuntimeError(f"drive wheels not found on {robot_path!r}; dofs={names}")

    dt = float(world.get_physics_dt())
    log(
        f"driving to ({goal[0]}, {goal[1]}) from ({spawn[0]}, {spawn[1]}) @ {args.spawn_yaw} rad; "
        f"budget {args.sim_time_max} sim-s = {int(args.sim_time_max / dt)} steps of {dt:.4f}s"
    )

    samples: list[tuple] = []
    reached_t = None
    t = 0.0
    try:
        while t < args.sim_time_max:
            contacts["t"] = t
            position, orientation = chassis.get_world_pose()
            x, y = float(position[0]), float(position[1])
            yaw = yaw_from_quat(orientation)
            distance = math.hypot(goal[0] - x, goal[1] - y)
            clearance = min_clearance(x, y, obstacles)
            if distance <= GOAL_RADIUS_M:
                reached_t = t
                samples.append((t, x, y, yaw, 0.0, 0.0, distance, clearance))
                break
            v, w = control(x, y, yaw, goal, obstacles)
            samples.append((t, x, y, yaw, v, w, distance, clearance))
            velocities = np.zeros(len(names))
            for i in left:
                velocities[i] = (v - 0.5 * w * WHEEL_TRACK_M) / WHEEL_RADIUS_M
            for i in right:
                velocities[i] = (v + 0.5 * w * WHEEL_TRACK_M) / WHEEL_RADIUS_M
            robot.set_joint_velocities(velocities)
            world.step(render=True)
            t += dt
    finally:
        robot.set_joint_velocities(np.zeros(len(names)))

    position, _ = chassis.get_world_pose()
    final_dist = math.hypot(goal[0] - float(position[0]), goal[1] - float(position[1]))
    if contacts["foreign_body"] is not None:
        log(
            f"WARN a non-robot prim touched a prop: {contacts['foreign_body']} "
            "(counted as a contact on purpose — see make_contact_log)"
        )

    total_events = sum(contacts["counts"].values())
    os.makedirs(OUT_DIR, exist_ok=True)
    write_trajectory(samples)
    write_json(
        OUT_CONTACTS,
        {
            "events": contacts["events"],
            "event_count": total_events,
            "events_truncated": total_events > len(contacts["events"]),
            "counts_by_type": contacts["counts"],
            "first_contact_t": contacts["first_contact_t"],
            "pairs": sorted(contacts["pairs"]),
        },
    )
    write_json(
        OUT_RUN,
        {
            # Exactly the nine axes, straight off argv — the oracle cross-checks them.
            "axes": {k: v for k, v in vars(args).items() if k != "gui"},
            "obstacles": [
                {
                    "kind": ob["kind"],
                    "asset": ob["asset"],
                    "x": round(ob["x"], 6),
                    "y": round(ob["y"], 6),
                    "yaw": round(ob["yaw"], 6),
                    "scale": ob["scale"],
                    "radius": round(ob["radius"], 6),
                }
                for ob in obstacles
            ],
            "reached": reached_t is not None,
            "time_to_goal_s": None if reached_t is None else round(reached_t, 4),
            "final_dist_m": round(final_dist, 6),
            "sim_time_s": round(t, 4),
            "seed": os.environ.get("CV_SEED"),
            "wheel_radius_m": WHEEL_RADIUS_M,
            "track_m": WHEEL_TRACK_M,
        },
    )
    log(
        f"wrote {OUT_DIR}/: {len(samples)} samples over {t:.2f} sim-s, "
        f"reached={reached_t is not None}, final dist {final_dist:.3f} m, "
        f"{total_events} contact events"
    )

    # Drop the contact subscription while the physics interface is still alive.
    subscription = None  # noqa: F841


def main() -> int:
    consent_guard()
    args = parse_args()
    log(f"CV_SEED={os.environ.get('CV_SEED')} (recorded; nothing here is stochastic)")

    # ORDERING: SimulationApp first, every Isaac import after it.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": not args.gui})
    rc = 0
    try:
        run(simulation_app, args)
    except Exception as exc:
        print(f"ERROR case failed: {exc!r}", file=sys.stderr, flush=True)
        rc = EXIT_ERROR
    finally:
        # Standard close, no os._exit: this is also what makes the exit code useless
        # as a verdict (see the module docstring).
        simulation_app.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
