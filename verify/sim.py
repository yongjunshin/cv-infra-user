#!/usr/bin/env python3
"""verify/sim.py — one verification case: drive the sample Nova Carter, log its pose.

A STANDARD Isaac Sim standalone script. cv-infra never imports this file and knows
nothing about what it means; per PICT case it runs, inside the stock Isaac image,

    /isaac-sim/python.sh verify/sim.py --drive_v=0.2 --drive_t=5 --yaw_rate=0.0

with this repository checked out read-only at the working directory, `verify/out/`
overlaid read-write, and `CV_SEED` in the environment. Which scene, which robot,
how it is driven and what gets written are OURS — the platform only supplies the
runtime, the input space and the collection of `verify/out/`.

EXIT CODE IS NOT A VERDICT (platform gotcha G-62): `SimulationApp.close()` ends the
process with status 0 no matter what happened, and the stock `python.sh` squashes a
non-zero status to 1. pass/fail is decided afterwards by `verify/oracle.py` reading
`verify/out/trajectory.csv`. A non-zero exit here can therefore only mean "this case
ERRORed" — which is what the guards below are for, and why they run before boot.

ORDERING (hard rule): `SimulationApp(...)` is constructed BEFORE any `omni.*` /
`isaacsim.*` import — those modules do not exist until the app has booted. Hence the
stdlib-only imports at the top and the in-function Isaac imports further down.

HEADLESS: a CI container has no display, so headless is the default; `--gui` is for
a developer running this at a desktop. Booting with a GUI in CI hangs or crashes.
"""

# stdlib only up here — see ORDERING above.
import argparse
import csv
import math
import os
import sys

# The official ROS 2 navigation sample: warehouse + Nova Carter + its own ROS 2
# OmniGraphs (clock / TF / odom / sensors / cmd_vel), all in one asset. Resolved
# against the Isaac assets root, which the image knows how to reach.
SCENE_USD = "/Isaac/Samples/ROS2/Scenario/carter_warehouse_navigation.usd"

# Where the ground-truth pose is read. A candidate LIST, not a single path: NVIDIA's
# own prim naming has moved between asset revisions, and a list degrades to a loud
# "none of these exist" instead of a silently wrong pin.
CHASSIS_CANDIDATES = (
    "/World/Nova_Carter_ROS/chassis_link",
    "/World/Carter_ROS/chassis_link",
)

# Checkout-relative, because the platform mounts the case's output directory over
# exactly this path (and `cd`s to the checkout). Same path when run locally.
OUT_CSV = os.path.join("verify", "out", "trajectory.csv")

WARMUP_STEPS = 8  # physics/renderer are unreliable on the very first steps
EXIT_ERROR = 1
EXIT_NO_CONSENT = 3


def log(msg: str) -> None:
    print(f"[cv-carter] {msg}", flush=True)


def parse_args() -> argparse.Namespace:
    """The axes of verify/param_space.pict, one flag each (plus the local --gui).

    Strict parsing on purpose: an axis added to the PICT model without a flag here
    fails loudly on argv instead of being silently ignored.
    """
    p = argparse.ArgumentParser(description="carter drive case for cv-infra")
    p.add_argument("--drive_v", type=float, required=True, help="linear speed [m/s]")
    p.add_argument("--drive_t", type=float, required=True, help="drive duration [sim s]")
    p.add_argument("--yaw_rate", type=float, required=True, help="angular speed [rad/s]")
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


def roll_pitch(quat_wxyz) -> tuple[float, float]:
    """Roll/pitch [rad] from a (w, x, y, z) quaternion — stdlib math, no numpy."""
    w, x, y, z = (float(v) for v in quat_wxyz)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    return roll, math.asin(sin_pitch)


def make_ros_driver():
    """Publish `/cmd_vel` with the image's bundled rclpy — returns send/close, or None.

    ASSUMPTION, NOT MEASURED: that `import rclpy` works inside `python.sh` on the
    stock image without sourcing a ROS environment. The platform measured the pieces
    on its own derived image (`cv_infra/runner/ros_bridge.py`, 2026-07-08): the
    bundled Jazzy site at `/isaac-sim/exts/isaacsim.ros2.bridge*/jazzy/rclpy` makes
    rclpy importable, and the bridge's own shared libraries additionally need that
    ext's `lib` on `LD_LIBRARY_PATH` *at process start* (the loader snapshots it, so
    an in-process prepend is too late). Neither fact has been re-measured here, on
    the stock image, at this Isaac version. So this path is attempted, never assumed:
    any failure returns None and the caller drives the articulation directly.
    """
    import glob

    for site in sorted(glob.glob("/isaac-sim/exts*/isaacsim.ros2.bridge*/jazzy/rclpy")):
        if site not in sys.path:
            sys.path.insert(0, site)

    try:
        import rclpy  # noqa: PLC0415
        from geometry_msgs.msg import Twist  # noqa: PLC0415
    except Exception as exc:
        log(f"WARN rclpy unavailable ({exc!r}) — falling back to articulation drive")
        return None

    try:
        rclpy.init()
        node = rclpy.create_node("cv_verify_carter")
        publisher = node.create_publisher(Twist, "cmd_vel", 10)
    except Exception as exc:
        log(f"WARN rclpy node setup failed ({exc!r}) — falling back to articulation drive")
        return None

    def send(v: float, w: float) -> None:
        msg = Twist()
        msg.linear.x = v
        msg.angular.z = w
        publisher.publish(msg)

    def close() -> None:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    log("drive path: ros2 /cmd_vel (scene's own subscribe graph)")
    return send, close


# Fallback wheel geometry. UNMEASURED ASSUMPTION (published Nova Carter figures):
# a wheel radius of 0.14 m and a 0.41 m track. Only the fallback path uses them, and
# only to turn (v, yaw_rate) into wheel spin; if they are off, the robot drives
# proportionally slower/faster and the oracle's displacement check sees it.
WHEEL_RADIUS_M = 0.14
WHEEL_TRACK_M = 0.41


def make_articulation_driver(robot_prim_path: str):
    """Command the wheel joints directly — no ROS anywhere in the loop.

    The second path of this file, kept because the first one is an assumption. It
    matches wheel joints by name (`*wheel*`, split by `left`/`right`); an unexpected
    naming degrades to "drive straight, ignore yaw" with a warning rather than to a
    wrong command.
    """
    import numpy as np  # noqa: PLC0415 (legal after SimulationApp boot)
    from isaacsim.core.prims import SingleArticulation  # noqa: PLC0415

    robot = SingleArticulation(robot_prim_path)
    robot.initialize()
    names = list(robot.dof_names)
    wheels = [i for i, n in enumerate(names) if "wheel" in n.lower()]
    left = [i for i in wheels if "left" in names[i].lower()]
    right = [i for i in wheels if "right" in names[i].lower()]
    if not wheels:
        raise RuntimeError(f"no wheel joint found on {robot_prim_path!r}; dofs={names}")
    if not left or not right:
        log(
            f"WARN wheel joints not split into left/right ({[names[i] for i in wheels]}); "
            "yaw_rate will be ignored on this path"
        )

    def send(v: float, w: float) -> None:
        velocities = np.zeros(len(names))
        half = 0.5 * w * WHEEL_TRACK_M
        for i in wheels:
            velocities[i] = v / WHEEL_RADIUS_M
        for i in left:
            velocities[i] = (v - half) / WHEEL_RADIUS_M
        for i in right:
            velocities[i] = (v + half) / WHEEL_RADIUS_M
        robot.set_joint_velocities(velocities)

    def close() -> None:
        robot.set_joint_velocities(np.zeros(len(names)))

    log(f"drive path: articulation wheel velocities ({len(wheels)} joints)")
    return send, close


def run(simulation_app, args: argparse.Namespace) -> None:
    """Open the sample scene, drive for `drive_t` sim-seconds, write the trajectory."""
    # Isaac imports are legal only here — after SimulationApp booted.
    import omni.usd  # noqa: PLC0415
    from isaacsim.core.api import World  # noqa: PLC0415
    from isaacsim.core.prims import SingleXFormPrim  # noqa: PLC0415
    from isaacsim.core.utils.extensions import enable_extension  # noqa: PLC0415
    from isaacsim.core.utils.stage import is_stage_loading  # noqa: PLC0415
    from isaacsim.storage.native import get_assets_root_path  # noqa: PLC0415

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
    # The scene's own graphs speak ROS 2; without the bridge extension nothing is
    # subscribed to /cmd_vel and the ROS path below would publish into the void.
    bridge_ok = bool(enable_extension("isaacsim.ros2.bridge"))
    simulation_app.update()
    if not bridge_ok:
        log("WARN isaacsim.ros2.bridge did not enable")

    world.reset()
    for _ in range(WARMUP_STEPS):
        world.step(render=True)

    stage = omni.usd.get_context().get_stage()
    chassis_path = next(
        (p for p in CHASSIS_CANDIDATES if stage.GetPrimAtPath(p).IsValid()),
        None,
    )
    if chassis_path is None:
        roots = [str(p.GetPath()) for p in stage.GetPseudoRoot().GetAllChildren()]
        raise RuntimeError(
            f"chassis prim not found — tried {list(CHASSIS_CANDIDATES)}; stage roots: {roots} "
            "(sample asset naming changed?)"
        )
    chassis = SingleXFormPrim(chassis_path)
    robot_prim_path = chassis_path.rsplit("/", 1)[0]

    driver = make_ros_driver() if bridge_ok else None
    if driver is None:
        driver = make_articulation_driver(robot_prim_path)
    send, close_driver = driver

    dt = float(world.get_physics_dt())
    steps = max(1, int(round(args.drive_t / dt)))
    log(
        f"driving {args.drive_v} m/s, {args.yaw_rate} rad/s for {args.drive_t}s "
        f"= {steps} steps of {dt:.4f}s"
    )

    rows = []
    try:
        for step in range(steps):
            send(args.drive_v, args.yaw_rate)
            world.step(render=True)
            position, orientation = chassis.get_world_pose()
            roll, pitch = roll_pitch(orientation)
            rows.append(
                (
                    (step + 1) * dt,
                    float(position[0]),
                    float(position[1]),
                    float(position[2]),
                    roll,
                    pitch,
                )
            )
    finally:
        close_driver()

    os.makedirs(os.path.dirname(OUT_CSV), exist_ok=True)
    with open(OUT_CSV, "w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(("t", "x", "y", "z", "roll", "pitch"))
        for row in rows:
            writer.writerow(f"{value:.6f}" for value in row)

    travelled = math.hypot(rows[-1][1] - rows[0][1], rows[-1][2] - rows[0][2])
    log(f"wrote {OUT_CSV}: {len(rows)} samples, net displacement {travelled:.3f} m")
    if travelled < 1e-3:
        # Worth calling out: a robot that did not move at all is more often a drive
        # path that reached nobody than a robot fault. The chosen path is logged above
        # and the whole container log travels with the case artifacts.
        log("WARN the chassis did not move — check the drive path line above")


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
        # as a verdict (G-62, see the module docstring).
        simulation_app.close()
    return rc


if __name__ == "__main__":
    sys.exit(main())
