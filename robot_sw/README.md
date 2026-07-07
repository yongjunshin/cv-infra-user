# robot_sw — carter demo SUT (blackbox for cv-infra)

The **System Under Test**: a robot-SW developer's app image. cv-infra consumes this
container as a **blackbox** — it never modifies the SUT internals (REQ-EXEC-004/005) and
wires it into its `ros2` adapter over a shared docker-network DDS. Our only job is to make
the SUT "behave so it can be verified": expose the contract below and stay unmodified when
cv-infra drives it.

> Status: **Phase 2 assembly** (CPU build). Real DDS join + goal drive is exercised on the
> GPU workstation in a later Phase-2 cycle, which also confirms the DRAFT topic names (R7).

## What is assembled (do-not-reinvent)

| asset | source | pinned to |
|---|---|---|
| base image | `ros:jazzy-ros-base-noble` (ROS 2 Jazzy, ros-base, headless) | exact tag (digest on workstation, D-1) |
| Nav2 | apt `ros-jazzy-navigation2` + `ros-jazzy-nav2-bringup` | ARG version pins (`Dockerfile`) |
| 3D lidar -> 2D scan | apt `ros-jazzy-pointcloud-to-laserscan` | ARG version pin |
| Nav config + warehouse map | `carter_navigation` (Isaac `jazzy_ws`) | commit `50de0035…` (tag `IsaacSim-5.1.0`) |

`carter_navigation` bundles the static warehouse map (`carter_warehouse_navigation.{png,yaml}`)
and the Nav2 param set, which we **reuse verbatim** — no re-authored params.

## The blackbox contract (what this SUT exposes)

cv-infra can verify this SUT because it satisfies:

1. **Accepts `use_sim_time:=true`, driven by external `/clock`** (REQ-EXEC-003). The image's
   default CMD launches with `use_sim_time:=true`; cv-infra injects the clock. All nav2 nodes
   run on sim time.
2. **Exposes the goal interface `nav2_msgs/action/NavigateToPose` @ `/navigate_to_pose`**
   (REQ-EXEC-007) — provided by nav2's `bt_navigator`.
3. **Publishes `/cmd_vel` + subscribes sensor topics** (REQ-EXEC-006). The controller (via the
   collision monitor) publishes `/cmd_vel`; nav2 subscribes to the lidar + odometry.
4. **Headless** — the shipped launch is rviz-free by construction (M8 §3.9 D-O); no GUI/desktop
   layers are pulled in.

### DRAFT wiring (R7 — confirmed on the GPU workstation, cycle-3)

These are carter's upstream defaults, not yet validated against the live sim graph:

| role | topic / action | type |
|---|---|---|
| goal (server) | `/navigate_to_pose` | `nav2_msgs/action/NavigateToPose` |
| actuation (pub) | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| lidar (sub) | `/front_3d_lidar/lidar_points` -> `pointcloud_to_laserscan` -> `/scan` | `PointCloud2` -> `LaserScan` |
| odometry (sub) | `/chassis/odom` | `nav_msgs/msg/Odometry` |
| clock (sub) | `/clock` | `rosgraph_msgs/msg/Clock` |
| frames | `map` / `odom` / `base_link` | TF |

## Build & inspect (CPU, no GPU/Isaac needed)

```bash
docker build -t carter-sut robot_sw/                 # local tag (D-1 hybrid build)
# entrypoint sources ROS + the carter overlay:
docker run --rm --entrypoint bash carter-sut -lc \
  "source /opt/ros/jazzy/setup.bash && ros2 launch --help >/dev/null && echo LAUNCH_OK"
```

The image's default `CMD` brings up the headless Nav2 stack:
`ros2 launch /opt/robot_sw/launch/carter_navigation_headless.launch.py use_sim_time:=true rviz:=false`.

## What we do NOT do

- No cv-infra-specific hooks are injected into the SUT — it stays a blackbox (REQ-EXEC-005).
- The Scenario / `adapter_config` **schema** is platform-owned (M1). `scenarios/*.yaml` only
  instantiate it as a consumer; see `../scenarios/nova_carter_warehouse_goal.yaml` (DRAFT).
