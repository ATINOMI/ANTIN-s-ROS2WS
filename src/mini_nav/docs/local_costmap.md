# 滚动局部代价地图

`local_costmap_node` 根据 `/scan` 维护一张独立的 `odom` 局部图，发布到
`/mini_nav/local_costmap`。窗口大小和分辨率固定，机器人移动时原点按栅格分辨率
滚动；位移达到整格时更新原点，重叠部分保留，新进入的区域为未知。
该节点不修改 `/map` 或全局规划图，
固定墙体仍由静态规划代价地图约束。

模块归属：滚动观测算法在 core 的 `map/rolling_obstacle_grid`，共用点类型在 `nav_types/`；ROS 接入在 nodes 的 `map_manager/local_costmap_node`。同目录的 `collision_map.hpp`、`cloud_validation.hpp` 和 `costmap_display.hpp` 负责消息校验与显示转换；连续车体扫掠由 core 的 `collision_checker/` 执行。完整目录见 [架构说明](architecture.md)。

每帧扫描在扫描时间查询 `odom <- base_footprint` 和 `odom <- 扫描坐标系` 的 TF。
所有有效射线先清除空闲格，再标记有限距离的障碍端点，以免相邻射线擦除命中点。
随后使用与全局图相同的 Nav2 1.3.12 膨胀核：按格中心距离分组传播，
外半径向上取整到整格。未知格在 `inflate_around_unknown` 开启时成为完整膨胀源；
接收行为固定为官方默认 `inflate_unknown=false`，允许硬代价覆盖未知格、拒绝软代价。
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
| `safety_margin` | 0.02 m | 真实车体安全余量 |
| `observation_uncertainty` | 0.03 m | 连续激光端点的额外观测误差预算 |
| `inscribed_radius` | 0.22549849949589046 m | learning 官方圆足迹含 padding 的膨胀内切半径；0 沿用车体安全半径 |
| `inflation_radius` | 0.70 m | 确定障碍的软代价最远距离 |
| `cost_scaling_factor` | 3.0 /m | 软代价指数衰减系数 |
| `inflate_around_unknown` | `false` | 未知格是否作为完整膨胀源 |
| `obstacle_max_range` | 2.5 m | 标记障碍的最远距离 |
| `raytrace_max_range` | 3.0 m | 射线清除的最远距离 |
| `observation_persistence` | 2.0 s | 单个格子的观测有效期 |
| `max_scan_age` | 1.0 s | 整条扫描流的有效期 |
| `publish_frequency` | 5 Hz | 地图发布频率 |

0.24 m 半径来自 Waffle 碰撞体的平面外接圆，计算见[规划代价地图](planning_costmap.md#参数与显示)。
加上 0.02 m 安全余量后，真实车体安全半径仍为 0.26 m。
显示图的膨胀内切半径独立设为 0.22549849949589046 m，外半径 0.70 m、衰减 3.0，
与 learning 官方膨胀参数一致。节点同时从同一原始观测生成
`/mini_nav/local_safety_costmap`，其硬半径仍为 0.26 m。
两张图不会相互再膨胀，也不额外添加窗口外缘硬圈。

两个定位与 A* 启动文件都会启动该节点。RViz 配置
`rviz/localization_astar.rviz` 默认叠加显示 `Local Costmap` 和 `Planning Costmap`；
Fixed Frame 为 `map` 时需要定位节点提供 `map -> odom`。
局部图以 `odom` 为坐标系，窗口跟随机器人；全局规划图仍固定在 `map`。
显示和安全膨胀图用于观测和调试；生产跟踪器订阅原子的
`/mini_nav/local_collision_map`（`mini_nav_nodes/msg/CollisionMap`）。它同时携带
未膨胀观测覆盖格、连续端点、0.26 m 车体安全半径、0.03 m 额外端点误差预算和有效性。
激光命中格在覆盖图中为已观测，不扩大为整个障碍方格；连续端点保留并参与
0.29 m 最小间距精查。静态墙体及 3D collision cloud 仍按完整占据格面积保守检查。
射线清除、滚动和过期同步更新格与端点；端点后方未知格仍禁行。
快照 header 使用真实观测时间，发布定时器不能刷新旧观测的新鲜度。
跟踪器还检查 `/mini_nav/local_costmap_valid`。扫描超时后膨胀图与原始覆盖图均
恢复未知，快照无效且端点清空；固定墙体由全局原始静态碰撞图约束。

默认 0.03 m 取自当前仿真 2D LaserScan 的 Gaussian 标准差 0.01 m 的三倍；
odom 系使用原 0.02 m 车体余量。map 配准实测约 4 cm，超出该预算，
因此全局原始几何另加 0.03 m 定位预算，合计 0.05 m，并以同时间真值评估检查。
三倍标准差是统计预算，不是高斯噪声绝对界。更换传感器或定位环境须重新评估。

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
