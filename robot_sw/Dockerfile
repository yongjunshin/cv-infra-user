# robot_sw/Dockerfile — carter demo SUT (System Under Test) for cv-infra E2E.
#
# Role: a robot-SW developer's app image. cv-infra consumes this container as a
# BLACKBOX — it never modifies SUT internals (REQ-EXEC-004/005) and wires it into
# its `ros2` adapter over a shared docker network (DDS). This repo's job is only
# to make the SUT "behave so it can be verified" (SUT contract, see P2 notes).
#
# STATUS: Phase 0 scaffold — pinned BASE LAYER ONLY. The navigation stack is added
# in Phase 2 (see "P2:" placeholders). Do NOT add Nav2/carter_navigation here yet
# (DDS handshake must be green first — debugging-surface separation).
#
# Reproducibility (decision 2026-06-24-env-reproducibility-pinning §3; CLAUDE.md §5):
#   - Base image pinned to an EXACT tag — no floating `ros:jazzy`.
#   - `jazzy-ros-base-noble` = ROS 2 Jazzy, ros-base variant (no desktop/GUI),
#     Ubuntu Noble pinned explicitly. headless: carter runs `rviz:=false` in CI
#     (M8 §3.9), so no GUI/desktop layers are pulled in.
#   - The immutable `@sha256` digest pin is added in Phase 2, at first workstation
#     pull, once the digest is observed (LOCKED #2). Until then this exact tag is
#     the pin.
FROM ros:jazzy-ros-base-noble
# P2: FROM ros:jazzy-ros-base-noble@sha256:<digest observed on first workstation pull>

# Non-interactive apt for reproducible CI builds.
ARG DEBIAN_FRONTEND=noninteractive

WORKDIR /opt/robot_sw

# --- P2: assemble reusable navigation assets (do-not-reinvent — assemble, don't build) ---
#   - Nav2 (ros-jazzy-navigation2 + ros-jazzy-nav2-bringup)
#   - carter_navigation (Isaac jazzy_ws Nav config for Nova Carter)
#   - pointcloud_to_laserscan (3D lidar -> 2D scan for Nav2 costmaps)
#   - static map asset
#   Pin apt package versions where practical (decision §3).
#
# --- P2: SUT contract the blackbox must satisfy so cv-infra can verify it ---
#   - accept use_sim_time:=true, driven by external /clock          (REQ-EXEC-003)
#   - expose goal interface nav2_msgs/action/NavigateToPose
#       @ /navigate_to_pose                                          (REQ-EXEC-007)
#   - publish /cmd_vel + subscribe sensor topics                    (REQ-EXEC-006)
#   cv-infra injects use_sim_time at launch; this image stays unmodified by it.
#
# --- P2: COPY launch/params (headless, rviz:=false) + entrypoint bringing up Nav2 ---
# P2: CMD ["ros2", "launch", ...]   # no CMD yet — scaffold base layer only.
