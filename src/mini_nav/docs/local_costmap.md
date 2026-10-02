# 滚动局部代价地图

`local_costmap_node` 根据 `/scan` 维护一张独立的 `odom` 局部图，发布到
`/mini_nav/local_costmap`。窗口大小和分辨率固定，机器人移动时原点按栅格分辨率
滚动；位移达到整格时更新原点，重叠部分保留，新进入的区域为未知。
该节点不修改 `/map` 或全局规划图，
固定墙体仍由静态规划代价地图约束。

每帧扫描在扫描时间查询 `odom <- base_footprint` 和 `odom <- 扫描坐标系` 的 TF。
所有有效射线先清除空闲格，再标记有限距离的障碍端点，以免相邻射线擦除命中点。
随后对确定障碍做机器人半径、安全余量和衰减代价的膨胀；未知格仅在
`inflate_around_unknown` 开启时向已知格传播硬安全距离，从不生成软代价。
未被刷新超过
`observation_persistence` 秒的格子恢复为未知；整帧扫描超时或 TF 不可用时，
整张局部图恢复为未知，并将 `/mini_nav/local_costmap_valid` 发布为 `false`。
未知格在 OccupancyGrid 中为 `-1`，空闲格为 `0`，硬安全区为 `99`，
致命障碍为 `100`；软代价映射到 `1..98`。这只影响发布显示，内部代价不变。

主要参数在 `mini_nav_bringup/config/local_costmap.yaml`：

| 参数 | 默认值 | 含义 |
| --- | ---: | --- |
| `width`, `height` | 4.0 m | 局部窗口尺寸 |
| `resolution` | 0.05 m | 栅格分辨率 |
| `robot_radius` | 0.24 m | 机器人半径 |
| `safety_margin` | 0.02 m | 硬安全余量 |
| `inflation_radius` | 0.45 m | 确定障碍的软代价最远距离 |
| `cost_scaling_factor` | 10.0 /m | 软代价指数衰减系数 |
| `inflate_around_unknown` | `false` | 未知格是否向已知格传播硬安全区 |
| `obstacle_max_range` | 2.5 m | 标记障碍的最远距离 |
| `raytrace_max_range` | 3.0 m | 射线清除的最远距离 |
| `observation_persistence` | 2.0 s | 单个格子的观测有效期 |
| `max_scan_age` | 1.0 s | 整条扫描流的有效期 |
| `publish_frequency` | 5 Hz | 地图发布频率 |

0.24 m 半径来自 Waffle 碰撞体的平面外接圆，计算见[规划代价地图](planning_costmap.md#参数与显示)。
加上 0.02 m 安全余量后，硬禁行距离为 0.26 m；0.45 m 半径只限制确定障碍的
软代价范围，不会改变硬碰撞阈值。

两个定位与 A* 启动文件都会启动该节点。RViz 配置
`rviz/localization_astar.rviz` 默认叠加显示 `Local Costmap` 和 `Planning Costmap`；
Fixed Frame 为 `map` 时需要定位节点提供 `map -> odom`。
局部图以 `odom` 为坐标系，窗口跟随机器人；全局规划图仍固定在 `map`。
该图当前用于观测和调试，
停车控制尚未接入；后续控制器应同时检查地图有效状态及局部轨迹上的硬障碍，
并保留静态规划图对固定墙体的约束。

从工作区根目录启动自研定位示例：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup
source install/setup.bash
ros2 launch mini_nav_bringup mini_localization_astar.launch.py
```

在 RViz 中查看默认开启的 `Local Costmap` 图层及其随机器人移动的边界；可用
`ros2 topic echo /mini_nav/local_costmap_valid` 查看观测是否有效。

验证时可先记录机器人前方格子的局部图代价和 `/map` 数据，再在 Gazebo 中创建
临时箱子。箱子所在格应变为 `100`；移走后，后续扫描射线应恢复该格原有代价。
短距离移动机器人后，局部图原点应随 `odom` 位姿移动，而 `/map` 数据保持不变。

2026-09-29 的无界面 Gazebo 验收中，箱子加入前后 `(0.9, 0)` 格的显示代价
为 `51 -> 100 -> 51`，前向扫描在箱子存在时约为 0.96 m；机器人前进约 0.294 m 后，
窗口 x 原点从 -2.00 m 滚到 -1.75 m。切断扫描而保留时钟和里程计后，
有效标记变为 `false`，80 × 80 个格子全部变为未知。
