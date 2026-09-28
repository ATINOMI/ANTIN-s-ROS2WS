# Official `nav2_costmap_2d` reference snapshot

This directory is an unchanged upstream package copied from the official
[`ros-navigation/navigation2`](https://github.com/ros-navigation/navigation2)
repository, branch `jazzy`.

- Source revision: `f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`
- Snapshot date: 2026-09-28
- Upstream package: [`nav2_costmap_2d`](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_costmap_2d)
- Package licenses: BSD-3-Clause and Apache-2.0 (see `package.xml` and source headers)

`COLCON_IGNORE` keeps this snapshot out of the `mini_nav` build. It is a
reading reference, not a runtime dependency.

## Suggested reading order for obstacle clearance

1. `include/nav2_costmap_2d/cost_values.hpp`: lethal, inscribed, and unknown cost values.
2. `include/nav2_costmap_2d/inflation_layer.hpp` and `plugins/inflation_layer.cpp`: radius, cost decay, caching, and propagation.
3. `include/nav2_costmap_2d/footprint.hpp` and `src/footprint.cpp`: robot footprint and inscribed/circumscribed radii.
4. `plugins/static_layer.cpp` and `src/layered_costmap.cpp`: how static occupancy and inflated costs reach the planning map.

Keep upstream copyright and license notices with any adapted code.
