# Official `nav2_amcl` reference snapshot

This directory is a learning reference copied from the official
[`ros-navigation/navigation2`](https://github.com/ros-navigation/navigation2)
repository, branch `jazzy`.

- Source revision: `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`
- Snapshot date: 2026-08-27
- Upstream package: [`nav2_amcl`](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_amcl)
- Package license: `LGPL-2.1-or-later` (see `package.xml` and the preserved source headers)

The package is intentionally kept under `reference/` and marked with
`COLCON_IGNORE`. It is not built, installed, launched, or used as a runtime
dependency of `mini_nav`. The self-developed, ROS-independent localization
implementation remains under `mini_nav_core`.

## Suggested reading order

1. `include/nav2_amcl/amcl_node.hpp` and `src/amcl_node.cpp`: ROS 2 lifecycle,
   map/scan/TF interfaces, callbacks, and pose publication.
2. `include/nav2_amcl/pf/` and `src/pf/`: particle-filter data structures,
   weighting, resampling, and KD-tree support.
3. `include/nav2_amcl/sensors/laser/` and `src/sensors/laser/`: laser models.
4. `include/nav2_amcl/motion_model/` and `src/motion_model/`: odometry motion
   models.
5. `include/nav2_amcl/map/` and `src/map/`: map representation and distance
   calculations.

Do not remove upstream copyright or license notices when studying or adapting
code. Any adapted implementation should remain clearly separated from this
snapshot and carry the applicable license information.
