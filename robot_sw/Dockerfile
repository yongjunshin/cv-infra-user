# robot_sw/Dockerfile — carter demo SUT (System Under Test) for cv-infra E2E.
#
# Role: a robot-SW developer's app image. cv-infra consumes this container as a
# BLACKBOX — it never modifies SUT internals (REQ-EXEC-004/005) and wires it into
# its `ros2` adapter over a shared docker network (DDS). This repo's job is only
# to make the SUT "behave so it can be verified" (SUT contract, see robot_sw/README.md).
#
# STATUS: Phase 2 — navigation stack ASSEMBLED (do-not-reinvent). The DDS handshake
# (DoD-P1-05) is green, so Nav2 assembly is now unblocked. This image builds on CPU
# (no GPU/Isaac needed to build); real DDS join + goal drive is exercised on the GPU
# workstation in a later Phase-2 cycle (R7 topic discovery finalizes the draft below).
#
# Reproducibility (decision 2026-06-24-env-reproducibility-pinning §3; CLAUDE.md §5,§2-7):
#   - Base image pinned to an EXACT tag — no floating `ros:jazzy`.
#   - `jazzy-ros-base-noble` = ROS 2 Jazzy, ros-base variant (no desktop/GUI),
#     Ubuntu Noble pinned explicitly. headless: carter runs rviz-free in CI
#     (M8 §3.9), so no GUI/desktop layers are pulled in.
#   - The immutable `@sha256` digest pin below was MEASURED at the first WORKSTATION
#     build (D-1, LOCKED #2) — build log #5, 2026-07-07, per
#     reports/user-2026-07-07-seam2-carter-topic-inventory.md §2 (fulfilled cycle-3).
#   - apt package versions are pinned via ARG (observed 2026-07-07). CAVEAT: the ROS
#     apt repo (packages.ros.org) rotates the datestamp suffix; if these exact
#     versions have been superseded when the workstation rebuilds (cycle-3), refresh
#     the ARG defaults (or pass --build-arg). The base-image digest is the immutable
#     reproducibility anchor; these pins are a best-effort snapshot (§3.3 "where practical").
FROM ros:jazzy-ros-base-noble@sha256:31daab66eef9139933379fb67159449944f4e2dcf2e22c2d12cc715f29873e0f
# P2 digest pin FULFILLED (cycle-3): measured at the first workstation build (D-1).

# Non-interactive apt for reproducible CI builds.
ARG DEBIAN_FRONTEND=noninteractive

# --- Pinned apt versions (observed 2026-07-07 on packages.ros.org; see CAVEAT above) ---
ARG NAVIGATION2_VERSION=1.3.12-1noble.20260615.181551
ARG NAV2_BRINGUP_VERSION=1.3.12-1noble.20260616.082701
ARG PC2LS_VERSION=2.0.2-3noble.20260615.151828

# --- P2: assemble reusable navigation assets (do-not-reinvent — assemble, don't build) ---
#   - Nav2 (ros-jazzy-navigation2 metapackage: runtime nav2 servers/BT/costmap/lifecycle)
#   - pointcloud_to_laserscan (3D lidar PointCloud2 -> 2D LaserScan for Nav2 costmaps)
#   - nav2_bringup LAUNCH FILES ONLY (bringup/navigation/localization composition, reused)
#   --no-install-recommends avoids unpinned floating Recommends (GOTCHAS G-11).
#   Transitive nav2 component Depends resolve from the same repo snapshot at build time.
#
# SLIM + SPLIT (T3 p5c5, decision 2026-07-22-p5-transport-approach B-A): the former SINGLE
# `apt-get install navigation2 nav2-bringup pointcloud-to-laserscan` produced one 2.79GB
# uncompressed layer -> ~784.5MB compressed GHCR blob that could not be pulled (infinite
# stall ~85MB on the target network; a single blob has no intra-layer resume). Root cause
# MEASURED on the workstation: `ros-jazzy-nav2-bringup` HARD-Depends the full Gazebo demo
# sim (nav2-minimal-tb3/tb4-sim, ros-gz-sim/bridge, slam-toolbox -> gz-*-vendor/ogre/
# dartsim/qt5-quick/sdformat/ruby/poco ... = 313-pkg delta, ~0.72GB NET dropped) which a
# HEADLESS SUT driven by EXTERNAL Isaac Sim never launches. (rviz2/mesa/llvm/vtk are NOT
# dropped here — also held by navigation2's rviz-plugins/PCL; see README residual ceiling.)
# Two levers, both applied below:
#   (1) SLIM  — drop the Gazebo closure: install nav2_bringup's 2.2MB launch package
#               WITHOUT its sim Depends (`dpkg --force-depends`; those tb3/tb4/gz/slam
#               launch paths are unused — default bringup path is slam:=False + external
#               /clock). Everything our headless launch runs is in `navigation2`.
#   (2) SPLIT — 3 pinned RUN layers (each an independently pull/resume-able blob) instead
#               of one fat blob; NO new per-subpackage pins (keeps §2-7 reproducibility).
# Per-layer sizing guide + degraded-mode fallback: robot_sw/README.md.

# Layer 1/3: pointcloud_to_laserscan (+ its PCL closure).
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-pointcloud-to-laserscan="${PC2LS_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 2/3: runtime Nav2 stack (navigation2 metapackage). navigation2's Depends carry
#   NO Gazebo (MEASURED) — pure runtime: bt_navigator/controller/planner/smoother/
#   behaviors/collision-monitor/costmap-2d/map-server/amcl/lifecycle-manager/docking.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-navigation2="${NAVIGATION2_VERSION}" \
    && rm -rf /var/lib/apt/lists/*

# Layer 3/3: nav2_bringup LAUNCH package only (bringup/navigation/localization launch +
#   default params/BT XML that our headless launch composes — do-not-reinvent). Installed
#   WITHOUT its Gazebo/sim hard-Depends (the tb3/tb4 gz demos + slam_toolbox), which the
#   external-Isaac-Sim headless SUT never launches. The unmet-dep marks are cosmetic for a
#   frozen leaf image; every node bringup_launch.py actually starts lives in `navigation2`.
RUN apt-get update \
    && apt-get download ros-jazzy-nav2-bringup="${NAV2_BRINGUP_VERSION}" \
    && dpkg --install --force-depends ./ros-jazzy-nav2-bringup_*.deb \
    && rm -f ./ros-jazzy-nav2-bringup_*.deb \
    && rm -rf /var/lib/apt/lists/*

# --- P2: assemble carter_navigation (Nova Carter Nav config + static warehouse map) ---
#   Not published to apt; assembled from the Isaac Sim ros_workspaces repo at a PINNED
#   COMMIT (= tag IsaacSim-5.1.0, matching CLAUDE.md §5 Isaac Sim 5.1.0). This package is
#   install-only (ament_cmake installs launch/params/maps/rviz2 dirs — no compilation).
#   It bundles maps/carter_warehouse_navigation.{png,yaml} (the static warehouse map) and
#   params/carter_navigation_params.yaml (the Nav2 param set we reuse verbatim).
ARG CARTER_NAV_REPO=https://github.com/isaac-sim/IsaacSim-ros_workspaces.git
ARG CARTER_NAV_COMMIT=50de00358f220d790d17050c6368cfe9a9cb9f51
RUN mkdir -p /opt/carter_ws/_src && cd /opt/carter_ws/_src \
    && git init -q \
    && git remote add origin "${CARTER_NAV_REPO}" \
    && git config core.sparseCheckout true \
    && git sparse-checkout init --cone \
    && git sparse-checkout set jazzy_ws/src/navigation/carter_navigation \
    && git fetch -q --depth 1 origin "${CARTER_NAV_COMMIT}" \
    && git checkout -q FETCH_HEAD \
    && cd /opt/carter_ws \
    && . /opt/ros/jazzy/setup.sh \
    && colcon build --merge-install --install-base /opt/carter_ws/install \
         --base-paths /opt/carter_ws/_src/jazzy_ws/src/navigation/carter_navigation \
    && rm -rf /opt/carter_ws/_src /opt/carter_ws/build /opt/carter_ws/log

WORKDIR /opt/robot_sw

# --- P2: headless (rviz-free) Nav2 bringup launch + entrypoint ---
#   The upstream carter_navigation.launch.py unconditionally starts RViz (no rviz arg),
#   so we ship a thin headless launch (nav-only, M8 §3.9 "or nav-only launch") that
#   composes nav2_bringup + carter's warehouse map + carter's params + pointcloud_to_laserscan.
COPY launch/ /opt/robot_sw/launch/
COPY ros_entrypoint.sh /opt/robot_sw/ros_entrypoint.sh
RUN chmod +x /opt/robot_sw/ros_entrypoint.sh

# --- SUT contract the blackbox satisfies so cv-infra can verify it (see robot_sw/README.md) ---
#   - accept use_sim_time:=true, driven by external /clock          (REQ-EXEC-003)
#   - expose goal interface nav2_msgs/action/NavigateToPose
#       @ /navigate_to_pose   (nav2 bt_navigator)                    (REQ-EXEC-007)
#   - publish /cmd_vel + subscribe sensor topics                    (REQ-EXEC-006)
#   cv-infra injects use_sim_time at launch; this image stays unmodified by it (REQ-EXEC-005).
ENTRYPOINT ["/opt/robot_sw/ros_entrypoint.sh"]
CMD ["ros2", "launch", "/opt/robot_sw/launch/carter_navigation_headless.launch.py", "use_sim_time:=true", "rviz:=false"]
