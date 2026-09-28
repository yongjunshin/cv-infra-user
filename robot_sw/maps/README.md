# maps/ — the robot's own static map of the warehouse

The occupancy map `robot_sw/driver.py` knows the warehouse by. It is the robot's asset —
what it has in memory before any mission — so it lives with the robot software, not with
the test harness. It contains the building (walls, shelves); it does NOT contain the
props a test case puts in the slots: those the robot has to see with its lidar.

| file | sha256 |
|---|---|
| `carter_warehouse_navigation.yaml` | `6898496a872b7831a91e867eb08b67eda07bd91c6f2313053268d7a591e6fd70` |
| `carter_warehouse_navigation.png` | `dd2f5e382a5f331866becaeaffb391a7e46b595873bf25c9cbb4e280ec261b8e` |

Both are **byte-identical to upstream** (`sha256sum` them to check):
`https://github.com/isaac-sim/IsaacSim-ros_workspaces`, commit
`50de00358f220d790d17050c6368cfe9a9cb9f51` (tag `IsaacSim-5.1.0`),
`jazzy_ws/src/navigation/carter_navigation/maps/`. Apache-2.0. It is the map NVIDIA
publishes for the very scene the harness opens (`carter_warehouse_navigation.usd`):
map frame = world frame, 0.05 m/px, origin (-11.975, -17.975).
