# carter_navigation_headless.launch.py — headless (rviz-free) Nav2 bringup for the
# carter demo SUT. cv-infra runs this in a headless GPU CI runner, so RViz/GUI must
# NOT start (M8 §3.9 D-O: rviz in headless CI is unnecessary + a failure source).
#
# do-not-reinvent: this is assembly glue, NOT a reimplementation. It composes reused
# assets verbatim:
#   - nav2_bringup/bringup_launch.py  (the full Nav2 stack — apt ros-jazzy-nav2-bringup)
#   - carter_navigation warehouse map + Nav2 params (assembled from Isaac jazzy_ws)
#   - pointcloud_to_laserscan node    (3D lidar PointCloud2 -> 2D LaserScan)
# It mirrors the upstream carter_navigation.launch.py EXCEPT it drops the RViz include.
#
# SUT contract surface exposed by this launch (see robot_sw/README.md):
#   - accepts use_sim_time (default True) -> external /clock            (REQ-EXEC-003)
#   - nav2 bt_navigator exposes nav2_msgs/action/NavigateToPose
#       @ /navigate_to_pose                                            (REQ-EXEC-007)
#   - subscribes sensor topics + publishes /cmd_vel                    (REQ-EXEC-006)
#
# DRAFT (R7): the sensor input topic (/front_3d_lidar/lidar_points) and target_frame
# are carter's upstream defaults. The REAL topics Isaac Sim publishes are confirmed on
# the GPU workstation in a later Phase-2 cycle; do not treat these as final.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    carter_share = get_package_share_directory("carter_navigation")
    nav2_bringup_launch_dir = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="True")
    map_yaml = LaunchConfiguration(
        "map",
        default=os.path.join(carter_share, "maps", "carter_warehouse_navigation.yaml"),
    )
    params_file = LaunchConfiguration(
        "params_file",
        default=os.path.join(carter_share, "params", "carter_navigation_params.yaml"),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="True",
                description="Use the external Isaac Sim /clock (REQ-EXEC-003).",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=map_yaml,
                description="Static warehouse map (assembled carter_navigation).",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=params_file,
                description="Nav2 params (assembled carter_navigation).",
            ),
            # Accepted for SUT-contract compatibility (M8 §3.9 documents `rviz:=false`).
            # This launch is headless by construction: RViz is never started regardless.
            DeclareLaunchArgument(
                "rviz",
                default_value="False",
                description="No-op: headless launch never starts RViz.",
            ),
            # Full Nav2 stack (bt_navigator -> /navigate_to_pose, controller -> /cmd_vel).
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_launch_dir, "bringup_launch.py")
                ),
                launch_arguments={
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                }.items(),
            ),
            # 3D lidar PointCloud2 -> 2D LaserScan on /scan (carter's upstream config).
            # DRAFT topic name (R7) — confirmed against sim on the GPU workstation.
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                remappings=[
                    ("cloud_in", "/front_3d_lidar/lidar_points"),
                    ("scan", "/scan"),
                ],
                parameters=[
                    {
                        "target_frame": "front_3d_lidar",
                        "transform_tolerance": 0.01,
                        "min_height": -0.4,
                        "max_height": 1.5,
                        "angle_min": -1.5708,  # -pi/2
                        "angle_max": 1.5708,  # pi/2
                        "angle_increment": 0.0087,  # pi/360
                        "scan_time": 0.3333,
                        "range_min": 0.05,
                        "range_max": 100.0,
                        "use_inf": True,
                        "inf_epsilon": 1.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )
