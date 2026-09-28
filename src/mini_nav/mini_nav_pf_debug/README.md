# Particle filter debug viewer

This package runs a separate copy of `mini_nav_core::localization::ParticleFilter`
against the existing `/map`, `/initialpose`, `/scan`, and TF streams. It publishes
no pose, velocity, or TF, so the running AMCL node is unaffected. It uses the
same model settings from `mini_nav_bringup/config/amcl_waffle.yaml` by default.

Start the normal `mini_localization_astar.launch.py` simulation first. In a
second terminal, use the same isolated ROS domain and run:

```bash
cd /home/a/ros2_ws
source src/mini_nav/scripts/env_mini_nav.sh
ros2 launch mini_nav_pf_debug pf_debug.launch.py
```

After this node reports that it has received the map, set an initial pose in
RViz. The first scan produces the score, normalized weight, resampling, and
estimate stages. Further updates follow the normal AMCL movement thresholds;
move the robot to capture more rounds. Open `/tmp/mini_nav_pf_debug.html` in a
browser and reload it after more updates arrive. A new initial pose starts a new
trace. `max_updates:=15` and `output_file:=/tmp/my_trace.html` are available
launch arguments.

The viewer shows each particle's position, heading and weight, the effective
sample size (ESS), and the estimated pose and standard deviations. It is an
independent diagnostic filter, not a recording of the main AMCL node's private
particles. Callback timing can make individual particles differ. The current
AMCL implementation computes an estimate after each usable scan; it does not
have a separate convergence decision, so the viewer does not claim that an
estimate is a confirmed pose.
