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
browser and reload it after more updates arrive. A new initial pose adds another
selectable session to the same HTML. The most recent five sessions are retained
by default; `max_sessions:=8`, `max_updates:=15`, and
`output_file:=/tmp/my_trace.html` are available launch arguments.

The viewer shows each particle's position, heading and weight, the effective
sample size (ESS), and the estimated pose and standard deviations. It is an
animated replay: initialization reveals particles in array order, and scoring
colors them in the same order used by both laser models. During resampling it
shows the actual systematic sampling offset, cumulative weight intervals,
selected source indices, eliminated sources, and any recovery injections.
The estimated pose gets a settling marker animation. Playback has phase
scrubbing and speeds from 0.25x to 16x. The data selector also accepts multiple
exported JSON files or generated HTML files. Older HTML files can be imported,
but their resampling source indices were not recorded and cannot be reconstructed.

This package is an
independent diagnostic filter, not a recording of the main AMCL node's private
particles. Callback timing can make individual particles differ. The current
AMCL implementation computes an estimate after each usable scan; it does not
have a separate convergence decision, so the viewer does not claim that an
estimate is a confirmed pose.
