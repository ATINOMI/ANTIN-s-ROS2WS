# Repository Guidelines

## Project Structure & Module Organization

This is a ROS 2 Jazzy `colcon` workspace. Packages live under `src/` and are independently buildable. Most C++ packages use `include/<package>/`, `src/`, `launch/`, `config/`, and `test/`. Python packages keep their module and `setup.py` together. Shared runtime assets are at the workspace root: `maps/`, `launch/`, `config/`, and `scripts/`.

`src/mini_nav/` is the self-built navigation learning project: `mini_nav_core` contains map and A* logic, `mini_nav_nodes` contains ROS nodes, and `mini_nav_bringup` owns launch/configuration assets. `nav2_learning`, `nav2_stvl_demo`, and `navigation2_tutorials` are reference and demonstration packages; do not make production code depend on their private files.

## Build, Test, and Development Commands

Always source Jazzy, build from the workspace root, then source the overlay:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup
source install/setup.bash
colcon test --packages-select mini_nav_core --event-handlers console_cohesion+
colcon test-result --verbose
```

Use `colcon build --packages-select <package>` for focused work. Launch examples with `ros2 launch <package> <file>.launch.py`. For simulator work, keep the documented `ROS_DOMAIN_ID` and `GZ_PARTITION` values isolated per example.

## Coding Style & Naming Conventions

Use C++17, four-space indentation, braces on the same line, and compiler warnings enabled (`-Wall -Wextra -Wpedantic`). Use `snake_case` for packages, nodes, topics, parameters, launch files, and frames; use `PascalCase` for ROS interface files. Keep ROS-independent algorithms in `*_core`; keep message conversion, parameters, and publishers/subscribers in node packages.

Declare ROS parameters with explicit types and validate numeric limits. Specify QoS deliberately. Use package-share lookups in launch files instead of workspace-specific absolute paths. Never publish a static `map -> odom` transform as a substitute for localization.

## Testing Guidelines

Add focused `ament_cmake_gtest` tests under `test/` for C++ logic; name files `test_<unit>.cpp`. Cover valid behavior and boundary cases such as out-of-map coordinates, obstacles, and no-path results. Run tests for every changed package before opening a pull request. Validate launch changes with `ros2 launch ... --show-args` before running a simulator.

## Commit & Pull Request Guidelines

Use short, imperative commit titles consistent with history, e.g. `Add mini_nav A-star navigation demo`. Keep a commit limited to one logical change. Pull requests should state affected packages, commands/tests run, launch or parameter changes, and simulator screenshots or logs when behavior is visual. Do not include build, install, log, or editor-generated files.
