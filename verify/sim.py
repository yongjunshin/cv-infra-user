#!/isaac-sim/python.sh
"""verify/sim.py — one verification case: can this robot's own software reach the goal?

A STANDARD Isaac Sim standalone script, and ONLY a test harness. cv-infra never imports
this file and knows nothing about what it means; per PICT case it runs, inside the stock
Isaac image,

    verify/sim.py --start=S1 --goal=G3 --slot_a=empty --slot_b=cardbox --slot_c=empty \
      --slot_d=barrel --slot_e=empty

through a `/bin/sh -lc 'exec "$0" "$@"'` wrapper — this file is the executable
entrypoint and the shebang above, not the platform, picks the interpreter — with this
repository checked out read-only at the working directory, `verify/out/` overlaid
read-write, and `CV_SEED` in the environment.

WHAT ONE CASE IS: build the world the axes name (the robot on one of three start
points, one of three goals, a stock warehouse prop in each occupied slot of a fixed
row), hand the goal to the robot's software, and then only WATCH: every physics step
the harness gives `robot_sw/straight_driver.py` its pose, turns the (v, w) it answers
into wheel spin, and records where the robot went and what it touched. The episode ends
when the robot is within 0.30 m of the goal, 1 s after its first collision, or after
SIM_TIME_MAX_S sim-seconds. `verify/oracle.py` turns the three files this writes into
{reached_goal, collision_free, …}.

THE HARNESS ADDS NO INTELLIGENCE. It does not steer, avoid, slow down or plan; the
robot software gets the goal and its own pose and nothing else — in particular not the
obstacle positions. What the robot can do is what `robot_sw/` can do, and a red case is
that software's result, not this file's.

EXIT CODE IS NOT A VERDICT (platform gotcha): `SimulationApp.close()` ends the process
with status 0 no matter what happened, and the stock `python.sh` squashes a non-zero
status to 1. A non-zero exit here can therefore only mean "this case ERRORed". Everything
the oracle needs is written BEFORE `close()`.

ORDERING (hard rule): `SimulationApp(...)` is constructed BEFORE any `omni.*` /
`isaacsim.*` / `pxr.*` / `numpy` import — those modules do not exist (or are not the
image's) until the app has booted. Hence the stdlib-only imports at the top and the
in-function Isaac imports further down.

HEADLESS: a CI container has no display, so headless is the default; `--gui` is for a
developer running this at a desktop. Booting with a GUI in CI hangs or crashes.
"""

# stdlib only up here — see ORDERING above.
import argparse
import csv
import hashlib
import importlib.util
import json
import math
import os
import sys

# The official ROS 2 navigation sample: warehouse + Nova Carter, all in one asset.
# Resolved against the Isaac assets root, which the image knows how to reach. We open it
# for its warehouse and its robot only — the scene's own ROS 2 OmniGraphs stay idle.
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

# The software under test — the robot's, not the harness's (see robot_sw/). Loaded from
# the checkout by path, and its sha256 goes into run.json so every verdict names the
# exact robot code it judged.
ROBOT_SW = os.path.join("robot_sw", "straight_driver.py")

# --------------------------------------------------------------------------------------
# The layout. Map frame (= world frame; the scene's nav2 map says so), metres. The area
# x in [-8, -2], y in [-1.5, 6.5] is open floor: the map shows >= 1.2 m of clearance
# everywhere in it, and the straight drives between the start and goal rows touched
# nothing but the floor (MEASURED 2026-09-28, probe drives on this image).
#
# The geometry is chosen so that a straight start->goal line crosses the slot row at
# EXACTLY one slot centre, and clears every other slot footprint by >= 0.27 m (robot
# half-width 0.24 m chassis / ~0.26 m wheels, both MEASURED). A straight driver therefore
# passes iff the one slot on its line is empty — no case hinges on centimetres:
#   slot a: S1->G1 · slot b: S1->G2, S2->G1 · slot c: S1->G3, S2->G2, S3->G1
#   slot d: S2->G3, S3->G2 · slot e: S3->G3
# --------------------------------------------------------------------------------------
START_YAW_RAD = math.pi / 2  # every start faces up the floor, toward the goal row
STARTS = {"S1": (-7.5, -1.0), "S2": (-5.0, -1.0), "S3": (-2.5, -1.0)}
GOALS = {"G1": (-7.5, 6.0), "G2": (-5.0, 6.0), "G3": (-2.5, 6.0)}
SLOTS = {
    "a": (-7.5, 2.5),
    "b": (-6.25, 2.5),
    "c": (-5.0, 2.5),
    "d": (-3.75, 2.5),
    "e": (-2.5, 2.5),
}

# Stock Simple_Warehouse props: (asset, x size, y size) in metres, placed axis-aligned.
# Sizes MEASURED on this image with a BBoxCache over a freshly referenced prim; min_z is
# 0 for both, so translating to z = 0 stands them on the floor. Each carries CollisionAPI
# but NOT RigidBodyAPI: a static collider that does not get shoved aside. Mind the asset
# spelling — "Barel" has one r.
PROPS = {
    "cardbox": ("/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxA_01.usd", 0.70, 0.50),
    "barrel": ("/Isaac/Environments/Simple_Warehouse/Props/SM_BarelPlastic_A_01.usd", 0.60, 0.71),
}
EMPTY = "empty"
OBSTACLE_PREFIX = "/World/cv_obs_"  # + slot letter

# The robot BODY — hardware, not behaviour. Wheel radius and track are the scene's own
# authored values on its DifferentialController node, and so are the speed limits the
# body enforces on whatever the software commands (MEASURED 2026-09-23).
WHEEL_RADIUS_M = 0.14
WHEEL_TRACK_M = 0.413
MAX_LINEAR_MS = 1.0
MAX_ANGULAR_RADS = 1.2

# The episode — the test's rules, not the robot's.
SIM_TIME_MAX_S = 30.0  # the longest line (8.6 m) takes ~23 s at the driver's 0.4 m/s
GOAL_RADIUS_M = 0.30  # "reached"
COLLISION_TAIL_S = 1.0  # keep watching this long after the first collision, then stop
# A contact whose every point has |normal z| >= this is the floor carrying the robot;
# anything flatter is the robot running into something. MEASURED: floor contacts are
# exactly |nz| = 1.000, the wheels against a cardbox exactly 0.000.
SUPPORT_NZ = 0.5

# Checkout-relative, because the platform mounts the case's output directory over
# exactly this path (and `cd`s to the checkout). Same paths when run locally.
OUT_DIR = os.path.join("verify", "out")
OUT_TRAJECTORY = os.path.join(OUT_DIR, "trajectory.csv")
OUT_CONTACTS = os.path.join(OUT_DIR, "contacts.json")
OUT_RUN = os.path.join(OUT_DIR, "run.json")

WARMUP_STEPS = 8  # physics/renderer are unreliable on the very first steps
MAX_EVENTS = 200  # a pinned robot reports every step — cap the dump
EXIT_ERROR = 1
EXIT_NO_CONSENT = 3


def log(msg: str) -> None:
    print(f"[cv-carter] {msg}", flush=True)


def parse_args() -> argparse.Namespace:
    """The axes of verify/param_space.pict, one flag each (plus the local --gui).

    Strict parsing on purpose: an axis added to the PICT model without a flag here, or a
    value this harness has no meaning for, fails loudly on argv instead of being ignored.
    """
    p = argparse.ArgumentParser(description="carter straight-to-goal case for cv-infra")
    p.add_argument("--start", required=True, choices=sorted(STARTS), help="start point")
    p.add_argument("--goal", required=True, choices=sorted(GOALS), help="goal point")
    for slot in SLOTS:
        p.add_argument(
            f"--slot_{slot}", required=True, choices=[EMPTY, *sorted(PROPS)], help="slot content"
        )
    p.add_argument("--gui", action="store_true", help="show the viewport (desktop only)")
    return p.parse_args()


def consent_guard() -> None:
    """Refuse to boot Isaac without the operator's runtime consent.

    No acceptance literal is committed anywhere; the platform passes the operator's own
    `ACCEPT_EULA` through into the case container.
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
# The world and the observations, as plain functions. Nothing here touches Isaac.
# --------------------------------------------------------------------------------------


def yaw_from_quat(quat_wxyz) -> float:
    """Yaw [rad] from a (w, x, y, z) quaternion — stdlib math, no numpy."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def obstacles_for(args: argparse.Namespace) -> list[dict]:
    """The props of one case: one per occupied slot, in slot order."""
    obstacles = []
    for slot, (x, y) in SLOTS.items():
        kind = getattr(args, f"slot_{slot}")
        if kind == EMPTY:
            continue
        asset, size_x, size_y = PROPS[kind]
        obstacles.append(
            {"slot": slot, "kind": kind, "asset": asset, "x": x, "y": y, "size_x": size_x, "size_y": size_y}
        )
    return obstacles


def min_clearance(x: float, y: float, obstacles: list[dict]):
    """Distance [m] from the robot's axle centre to the nearest prop footprint, or None."""
    if not obstacles:
        return None
    best = math.inf
    for ob in obstacles:
        dx = max(abs(x - ob["x"]) - 0.5 * ob["size_x"], 0.0)
        dy = max(abs(y - ob["y"]) - 0.5 * ob["size_y"], 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def load_robot_sw(path: str):
    """The software under test, imported from the checkout by file path — (module, sha256)."""
    with open(path, "rb") as handle:
        digest = hashlib.sha256(handle.read()).hexdigest()
    spec = importlib.util.spec_from_file_location("robot_sw_under_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module, digest


def label_of(prim_path: str, obstacles: list[dict]) -> str:
    """What the robot touched, in the words of the case: `slot c cardbox` or the prim path."""
    if prim_path.startswith(OBSTACLE_PREFIX):
        slot = prim_path[len(OBSTACLE_PREFIX) :].split("/", 1)[0]
        kind = next((ob["kind"] for ob in obstacles if ob["slot"] == slot), "?")
        return f"slot {slot} {kind}"
    return prim_path


def make_contact_log(robot_path: str, obstacles: list[dict], to_sdf_path):
    """The PhysX contact callback and the record it fills — (callback, log).

    The report is attached to the ROBOT's bodies, so it hears everything the robot
    touches: the floor, the props, and anything else in the warehouse. The floor is told
    apart by the contact normal (see SUPPORT_NZ), so "collision" needs no list of what
    the scene contains. `to_sdf_path` is `PhysicsSchemaTools.intToSdfPath`, passed IN so
    this stays a stdlib function at module scope (see ORDERING). The caller keeps
    `log["t"]` current: the callback fires from inside `world.step()`.
    """
    prefix = robot_path + "/"
    log_ = {
        "t": 0.0,
        "first_collision": None,
        "collisions": [],
        "collision_event_count": 0,
        "counts_by_type": {},
        "support_surfaces": set(),
    }

    def on_contact(headers, data) -> None:
        for header in headers:
            actor0 = str(to_sdf_path(header.actor0))
            actor1 = str(to_sdf_path(header.actor1))
            mine0, mine1 = actor0.startswith(prefix), actor1.startswith(prefix)
            if mine0 == mine1:
                continue  # neither side is the robot, or both are
            body, other = (actor0, actor1) if mine0 else (actor1, actor0)
            if header.num_contact_data == 0:
                continue  # CONTACT_LOST carries no points — nothing new was touched
            points = range(header.contact_data_offset, header.contact_data_offset + header.num_contact_data)
            flattest = min(abs(float(data[k].normal[2])) for k in points)
            if flattest >= SUPPORT_NZ:
                log_["support_surfaces"].add(other)
                continue
            event_type = str(header.type).split(".")[-1]
            record = {
                "t": round(log_["t"], 4),
                "type": event_type,
                "body": body.rsplit("/", 1)[-1],
                "other": other,
                "label": label_of(other, obstacles),
                "min_abs_nz": round(flattest, 3),
            }
            log_["collision_event_count"] += 1
            log_["counts_by_type"][event_type] = log_["counts_by_type"].get(event_type, 0) + 1
            if log_["first_collision"] is None:
                log_["first_collision"] = record
            if len(log_["collisions"]) < MAX_EVENTS:
                log_["collisions"].append(record)

    return on_contact, log_


# --------------------------------------------------------------------------------------
# Output. Written before SimulationApp.close(), because the exit code carries nothing.
# --------------------------------------------------------------------------------------


def write_trajectory(samples: list[tuple]) -> None:
    """One row per physics step. An empty cell is JSON `null` to the oracle."""
    with open(OUT_TRAJECTORY, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("t", "x", "y", "yaw", "v_cmd", "w_cmd", "dist_to_goal", "min_clearance"))
        for row in samples:
            writer.writerow("" if value is None else f"{value:.6f}" for value in row)


def write_json(path: str, payload: dict) -> None:
    with open(path, "w") as handle:
        json.dump(payload, handle, indent=1, sort_keys=True)
        handle.write("\n")


def run(simulation_app, args: argparse.Namespace) -> None:
    """Build the case's world, let the robot software drive, write verify/out/."""
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

    start_x, start_y = STARTS[args.start]
    goal = GOALS[args.goal]
    obstacles = obstacles_for(args)
    robot_sw, robot_sw_sha = load_robot_sw(ROBOT_SW)
    log(f"robot software under test: {ROBOT_SW} sha256 {robot_sw_sha[:12]}")
    log(f"{args.start} ({start_x}, {start_y}) -> {args.goal} {goal}")
    for ob in obstacles:
        log(f"slot {ob['slot']}: {ob['kind']} at ({ob['x']}, {ob['y']})")

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
    # the articulation root (the chassis) itself before play breaks the robot.
    SingleXFormPrim(robot_path).set_world_pose(
        np.array([start_x, start_y, 0.0]),
        np.array([math.cos(0.5 * START_YAW_RAD), 0.0, 0.0, math.sin(0.5 * START_YAW_RAD)]),
    )

    # PROPS, also BEFORE world.reset(): their triangle-mesh colliders are baked at scene
    # init. A prop without a collider would make "did it touch?" meaningless — refuse it.
    for ob in obstacles:
        prim = stage.DefinePrim(f"{OBSTACLE_PREFIX}{ob['slot']}", "Xform")
        prim.GetReferences().AddReference(assets_root + ob["asset"])
        UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(ob["x"], ob["y"], 0.0))
        if not any(p.HasAPI(UsdPhysics.CollisionAPI) for p in Usd.PrimRange(prim)):
            raise RuntimeError(f"{ob['asset']!r} referenced no collider — cannot detect contact")

    # CONTACTS: a raw PhysX contact report on every rigid body of the ROBOT. (A
    # ContactSensor on the chassis missed a box the wheels hit first, and the tensor API
    # cannot see static colliders — both MEASURED.)
    bodies = [p for p in Usd.PrimRange(stage.GetPrimAtPath(robot_path)) if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    for body in bodies:
        PhysxSchema.PhysxContactReportAPI.Apply(body).CreateThresholdAttr().Set(0.0)
    on_contact, contacts = make_contact_log(robot_path, obstacles, PhysicsSchemaTools.intToSdfPath)
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

    # The mission: the goal, once. From here on the harness only reports the pose.
    driver = robot_sw.StraightDriver(*goal)
    dt = float(world.get_physics_dt())
    log(f"episode: budget {SIM_TIME_MAX_S} sim-s = {int(SIM_TIME_MAX_S / dt)} steps of {dt:.4f}s")

    samples: list[tuple] = []
    reached_t = None
    end_reason = "budget"
    t = 0.0
    try:
        while t < SIM_TIME_MAX_S:
            contacts["t"] = t
            position, orientation = chassis.get_world_pose()
            x, y = float(position[0]), float(position[1])
            yaw = yaw_from_quat(orientation)
            distance = math.hypot(goal[0] - x, goal[1] - y)
            clearance = min_clearance(x, y, obstacles)
            if distance <= GOAL_RADIUS_M:
                reached_t, end_reason = t, "reached"
                samples.append((t, x, y, yaw, 0.0, 0.0, distance, clearance))
                break
            first = contacts["first_collision"]
            if first is not None and t >= first["t"] + COLLISION_TAIL_S:
                end_reason = "collision"
                samples.append((t, x, y, yaw, 0.0, 0.0, distance, clearance))
                break
            # The robot software decides; the body only enforces its own speed limits.
            v, w = driver.step(x, y, yaw)
            v = max(-MAX_LINEAR_MS, min(MAX_LINEAR_MS, float(v)))
            w = max(-MAX_ANGULAR_RADS, min(MAX_ANGULAR_RADS, float(w)))
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

    os.makedirs(OUT_DIR, exist_ok=True)
    write_trajectory(samples)
    write_json(
        OUT_CONTACTS,
        {
            "first_collision": contacts["first_collision"],
            "collisions": contacts["collisions"],
            "collision_event_count": contacts["collision_event_count"],
            "collisions_truncated": contacts["collision_event_count"] > len(contacts["collisions"]),
            "counts_by_type": contacts["counts_by_type"],
            "support_surfaces": sorted(contacts["support_surfaces"]),
        },
    )
    write_json(
        OUT_RUN,
        {
            # Exactly the seven axes, straight off argv — the oracle cross-checks them.
            "axes": {k: v for k, v in vars(args).items() if k != "gui"},
            "start": {"name": args.start, "x": start_x, "y": start_y, "yaw": START_YAW_RAD},
            "goal": {"name": args.goal, "x": goal[0], "y": goal[1]},
            "obstacles": obstacles,
            "robot_sw": {"path": ROBOT_SW, "sha256": robot_sw_sha},
            "reached": reached_t is not None,
            "time_to_goal_s": None if reached_t is None else round(reached_t, 4),
            "collided": contacts["first_collision"] is not None,
            "end_reason": end_reason,
            "final_dist_m": round(final_dist, 6),
            "sim_time_s": round(t, 4),
            "budget_s": SIM_TIME_MAX_S,
            "seed": os.environ.get("CV_SEED"),
        },
    )
    log(
        f"wrote {OUT_DIR}/: {len(samples)} samples over {t:.2f} sim-s, end={end_reason}, "
        f"final dist {final_dist:.3f} m, {contacts['collision_event_count']} collision events"
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
        # Standard close, no os._exit: this is also what makes the exit code useless as
        # a verdict (see the module docstring).
        simulation_app.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
