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

## Pull-friendliness — per-layer size guide + degraded-mode fallback

The SUT image is pushed to GHCR and pulled onto the GPU runner during E2E. A **single fat
layer** is the enemy: a large blob has no intra-layer resume, so a flaky/slow network stalls
mid-blob and the whole pull hangs (measured p5c4: the former single Nav2 install layer =
2.79 GB uncompressed -> **~734 MiB gzip blob**, stalled ~85 MB on the target network with no
recovery). Decision `2026-07-22-p5-transport-approach` (B-A) — mitigate at the image, keep
each layer an independently pull/resume-able blob.

### Rule: never install into one fat layer

Split a big `apt-get install` into **pinned RUN layers** (each a separate, resumable blob) and
**drop headless-unneeded closures** where the packaging allows. Applied to this Dockerfile
(measured on the workstation, `docker history` before/after — see
`reports/user-2026-07-22-sut-slim-mcap.md`):

| | before (single layer) | after (slim + split) |
|---|---|---|
| Nav2 install layers | 1 | 3 (pc2ls · navigation2 · nav2_bringup) |
| max layer (uncompressed) | 2.79 GB | 2.07 GB (navigation2) |
| max layer (gzip pull blob) | **734 MiB** | **543 MiB** |
| other layers | — | 5.4 MB (pc2ls) + 4.1 MB (nav2_bringup), independent blobs |
| total image | 3.68 GB | **2.97 GB** |

- **Slim lever applied**: `ros-jazzy-nav2-bringup` HARD-Depends the full Gazebo demo sim
  (`nav2-minimal-tb3/tb4-sim`, `ros-gz-sim/bridge`, `slam-toolbox` -> `gz-*-vendor`/ogre/
  dartsim/qt5-quick/… = 313 pkgs, ~0.72 GB) that a **headless SUT driven by external Isaac
  Sim never launches**. We install its 2.2 MB launch package WITHOUT those sim Depends
  (`dpkg --install --force-depends`) — every node our launch actually starts lives in
  `navigation2`. The unmet-dep marks are cosmetic for a frozen leaf image (verified: launch
  still composes, all Nav2 executables present).
- **Residual ceiling (known)**: the remaining 543 MiB `navigation2` blob is still dominated by
  `nav2-rviz-plugins` -> rviz2/OGRE/mesa/**libllvm** (~0.38 GB) and PCL `libpcl-dev` ->
  boost-dev/VTK/opencv-dev (~0.5 GB), both **held by upstream metapackage/PCL hard-Depends**.
  Shedding them cleanly is blocked (removing a metapackage's hard-Dep breaks the metapackage;
  `apt autoremove` then refuses). Forcing it (stacked `dpkg --force-depends` + `apt-mark`
  surgery against ROS/PCL packaging) is over-engineering with runtime-breakage risk, so it is
  **not done** — the slim is a *partial* mitigation, not a full close of the pull wall.

### Degraded-mode fallback (MVP-normative on the target network)

Because slim alone does not drop the max blob under the stall threshold, the **normal MVP path**
is to **pre-stage the SUT image on the runner and reference it directly**, so `verify` skips the
GHCR pull entirely (validated green x4 in p5c4):

```bash
# on the self-hosted GPU runner: pre-stage once (any channel that completes),
# then scenarios reference the local ref — no CI-time GHCR pull:
docker pull ghcr.io/<ORG>/cv-infra-user/carter-sut@sha256:<digest>   # or load from a saved tar
docker tag  <that image>  carter-sut:p2      # local pre-staged ref used by scenarios
```

Revert to the full CI-build digest (pull-through) only once slim/split is proven end-to-end on
the target network (decision B-A carry-forward, p5c6+). Until then, degraded-mode is the
documented, supported deploy path (see also the redeploy manual, DoD-P5-08).

## What we do NOT do

- No cv-infra-specific hooks are injected into the SUT — it stays a blackbox (REQ-EXEC-005).
- The Scenario / `adapter_config` **schema** is platform-owned (M1). `scenarios/*.yaml` only
  instantiate it as a consumer; see `../scenarios/nova_carter_warehouse_goal.yaml` (DRAFT).
