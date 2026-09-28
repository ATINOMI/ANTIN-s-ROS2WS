# Official `nav2_smac_planner` reference snapshot

This directory is an unchanged upstream package copied from the official
[`ros-navigation/navigation2`](https://github.com/ros-navigation/navigation2)
repository, branch `jazzy`.

- Source revision: `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`
- Snapshot date: 2026-09-28
- Upstream package: [`nav2_smac_planner`](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smac_planner)
- Package license: Apache-2.0 (see `package.xml` and source headers)

`COLCON_IGNORE` keeps this snapshot out of the `mini_nav` build. It is a
reading reference, not a runtime dependency.

## Suggested reading order for cost-aware A*

1. `src/smac_planner_2d.cpp`: planning entry point and costmap connection.
2. `include/nav2_smac_planner/node_2d.hpp` and `src/node_2d.cpp`: collision checks and traversal cost from cell cost.
3. `include/nav2_smac_planner/a_star.hpp` and `src/a_star.cpp`: search loop and cost accumulation.
4. `include/nav2_smac_planner/collision_checker.hpp` and `src/collision_checker.cpp`: robot radius/footprint collision checks.

Keep upstream copyright and license notices with any adapted code.
