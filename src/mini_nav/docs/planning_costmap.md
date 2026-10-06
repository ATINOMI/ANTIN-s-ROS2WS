# 规划代价地图

`costmap_publisher_node` 从 `/map`（或 `map_file`）读取原始静态图，保留
`/mini_nav/map` 的原始占据信息，并另外发布 `/mini_nav/planning_costmap`。
自研 A* 使用后者规划；AMCL 仍使用原始 `/map`。收到新地图时会重新生成规划图。

2026-10-06 的模块划分中，地图和膨胀算法位于 core 的 `map/`，A* 与后处理位于 `navigator/planner/`，连续车体安全检查位于 `collision_checker/`。ROS 节点及地图消息校验位于 nodes 的 `map_manager/`。完整路径见 [架构说明](architecture.md)。

## 代价规则

- 膨胀核对齐 Nav2 1.3.12 `InflationLayer` 的全图更新；原始 `/map` 不会改写。
- 距离为格中心间的欧氏距离，`inscribed_radius` 为膨胀硬区半径；
  设为零时沿用 `robot_radius + safety_margin`。源格为 `254`，硬区内为 `253`。
- 外半径先按 `ceil(inflation_radius / resolution)` 向上取整到整格。
  软代价为 `static_cast<unsigned char>(252 * exp(-cost_scaling_factor *
  (distance - hard_radius)))`，允许截断到零。
- 使用距离分组、四邻域传播和首次访问来源规则；不能用所有障碍逐格取最大
  膨胀代价代替。已有非未知代价仍与所选来源代价取最大值。
- `inflate_around_unknown` 控制未知格是否作为完整膨胀源。接收行为固定为
  官方默认 `inflate_unknown=false`：未知格允许被 `253/254` 覆盖，拒绝软代价。
  未被覆盖的未知格仍是 `255`；原始地图始终保留未知信息。
- 膨胀核不再人为添加地图外缘硬圈，车体与图外的距离仍由原图连续扫掠检查。
- A* 走八邻域，直边长度为 `1`、斜边为 `√2`；步长乘以
  `1 + cost_travel_multiplier * cell_cost / 252`，启发式为八方向距离。
  生产入口在原始静态格面积、连续动态端点上检查节点和整条边，
  斜走时两侧正交位置也必须安全。显示图中的动态 `253/254` 不独立判死。
  车体基础安全半径为 `robot_radius + safety_margin = 0.26 m`，map 系再加
  `localization_uncertainty = 0.03 m`，得到 0.29 m；map 系动态端点再加
  `observation_uncertainty = 0.03 m`，得到 0.32 m。未知格面积和地图外边界始终禁行，
  此硬安全策略独立于显示层的未知膨胀开关。

膨胀实现参考固定版本 [Nav2 1.3.12 InflationLayer](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/plugins/inflation_layer.cpp)，
默认膨胀参数也已对齐 learning：官方 0.22 m 圆足迹经 0.01 m padding 后
内切半径为 0.22549849949589046 m，外半径 0.70 m、衰减 3.0。
真实车体外接圆与余量仍独立保留。回归测试直接调用已安装官方库，
默认地图使用官方真实圆足迹，要求全部 11536 格一致；还覆盖未知格、边缘和多障碍竞争。
A* 软代价对应
`reference/nav2_smac_planner_jazzy` 的 `Node2D::getTraversalCost`。
实现位于自研 `mini_nav_core`，运行时不依赖参考包。

## 参数与显示

两套定位加 A* 的 launch 均加载
`mini_nav_bringup/config/planning_costmap.yaml`：默认机器人半径 0.24 m、
额外安全余量 0.02 m、膨胀内切半径 0.22549849949589046 m、
膨胀半径 0.70 m、衰减系数 3.0、A* 软代价权重 2.0。
仿真入口实际生成的 `turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf` 中，
底盘碰撞盒为 0.265 × 0.265 m，中心相对
`base_footprint` 的水平偏移为 -0.064 m；最远角点半径为
`√((0.064 + 0.265/2)² + (0.265/2)²) = 0.2370 m`。TF 所用
`turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf` 的
0.266 × 0.266 m 碰撞盒算得 0.2377 m；轮、脚轮和激光雷达碰撞体均在此
半径内，因此 `robot_radius` 向上取整为 0.24 m。额外 0.02 m 使真实车体安全半径
为 0.26 m，比最远碰撞角点多约 0.022 m；隔离 SCURM 路线实测 map 配准误差最大约 4 cm，因此生产 map 系几何另加
0.03 m 定位预算，与原 0.02 m 合计预留 0.05 m；这是当前短程仿真的预算，
不是任意环境或实车保证。
不应把官方示例半径当成该偏置碰撞盒的外接圆。膨胀内切硬圈采用独立参数，
A* 与后处理采用基础 0.26 m 加显式的 map 定位预算，不能只凭显示图的软区判断能否通行。
若用独立节点启动，可用同名 `planning.*` ROS 参数覆盖。`robot_radius`
和 `inflation_radius` 必须为正，`safety_margin` 和权重必须为非负，且
`inscribed_radius` 必须为有限非负数，`inflation_radius` 不得小于车体安全半径或膨胀内切半径。
`planning.inflate_around_unknown` 默认为 `false`。两套定位 launch 还提供同名
`inflate_around_unknown` 启动参数，同时控制全局规划图和局部图；例如：

```bash
ros2 launch mini_nav_bringup mini_localization_astar.launch.py inflate_around_unknown:=true
```

参数在启动时读取，修改后需重启对应节点；关闭未知源传播仍允许硬代价覆盖未知格，
被覆盖后的 `253/254` 也不可通行。

RViz 的 **Planning Costmap** 显示订阅 `/mini_nav/planning_costmap`。
独立 A* 与定位视图均默认显示它；定位视图同时显示 `Local Costmap`，
距离场图层默认关闭。底图透明度 1.0、全局图 0.3、局部图 0.7，与官方视图一致。
发布给 RViz 的占据值与内部代价分开：
`253` 硬安全区显示为 `99`（青色），`254` 障碍显示为 `100`（紫色），
`255` 未知显示为 `-1`；`1..252` 软代价映射到 `1..98`。显示透明度默认
为 `0.3`，与官方示例的全局代价地图一致。规划先检查实际连续起点的完整车体。
所属格中心或连接不安全时，从八个邻接格中选取可以安全连接的中心，
同时作为 A* 搜索入口；首段使用全部当前原始扫描端点检查，保留实际位置。
实际车体不安全或没有安全连接时仍规划失败；不会缩小半径或越过障碍。
目标不可达时按下述目标容差寻找安全终点。

节点还从同一原始障碍图生成 `/mini_nav/planning_safety_costmap`，
其膨胀硬半径为 `robot_radius + safety_margin`，默认 0.26 m。
该图和 `/mini_nav/local_safety_costmap` 保留作调试。跟踪器改为订阅
`mini_nav_nodes/msg/CollisionMap` 类型的 `/mini_nav/static_collision_map` 与
`/mini_nav/local_collision_map`，对完整车体作连续扫掠；map 快照半径为 0.29 m、odom 快照为 0.26 m，
控制器独立核对半径与自身参数一致，显示硬圈不改变此约束。
`planning.localization_uncertainty` 与 `controller.localization_uncertainty` 必须一致；
额外预算不得靠显示膨胀参数代替。
map 碰撞快照携带规划起点检查使用的全部原始连续扫描端点和观测误差，
不受稳定观测门限过滤。实时端点以 `points_frame_id: odom` 保留扫描时刻的原始投影，
规划器和控制器用当前 `odom <- map` 检查路径扫掠；端点要求间距为
`robot_radius + safety_margin + observation_uncertainty`，默认 0.29 m。
静态格仍要求 0.29 m，包含 0.03 m 地图定位预算；实时相对观测不再次叠加此项。
在地图定位修正前后，机器人和观测的共同坐标变换不会改变相对距离。
直接提供 map 同帧端点的输入继续使用旧保守预算，默认要求 0.32 m。
空观测帧按 grid 同帧解释；未知或不受支持的观测帧不能被当作 odom；TF 或输入失效时仍停车。
消息新增观测帧字段，升级后须一起重启整套导航节点。
快照在发布时重新构建，保留观测时间；所需扫描、点云或 TF 失效时发布 `valid=false`。
默认 `planning.dynamic_policy: static_then_stable`：首轮路线以静态几何为硬约束，
动态扫描仍参与当前位置及首段检查和软代价；局部第一次有效命中即参与停车。
持续受阻触发 `AStarDynamic` 后，至少三次独立扫描、持续 0.3 s 的端点格加入
全局硬约束；观测间隔超过 1.5 s 重新累计。地图替换和定位纪元重置会清除统计。
`immediate` 策略则全程使用当前动态端点作为全局硬约束。

## 不可达目标与速度

`planning.goal_tolerance` 默认 **0.5 m**，必须为有限非负数，设为 `0` 禁用。
先尝试原目标栅格；若它位于障碍、未知格、硬安全区，车体余量不足或没有路径，
则从起点实际可达的安全格中选取距离原目标最近的终点。距离以目标栅格中心为基准，
使用欧氏距离；等距离候选优先选到达代价较低者。所有搜索边仍检查硬安全区、
斜向穿角和连续车体扫掠，目标容差不会放宽这些条件。容差内无可达点时发布空路径。
原目标可达时始终使用原目标；地图外的目标仍拒绝规划。
无论采用哪种动态策略，原目标和替代终点都须通过当前全部观测的连续碰撞检查。
`planning.goal_position_tolerance` 默认 **0.12 m**，必须有限且为正，
须不小于跟踪器的 `controller.goal_position_tolerance`。
终点检查在原安全半径上额外加上该停车容差，使整个允许停车的圆盘都安全；
它只约束终点选择，不改变途中车体半径或 RViz 的膨胀代价。
该保证针对规划快照，障碍后续移动或定位失效仍交给实时控制检查和任务重规划。

该行为参考官方 Jazzy [NavFnPlanner::makePlan](https://github.com/ros-navigation/navigation2/blob/f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501/nav2_navfn_planner/src/navfn_planner.cpp#L274-L310)：
官方先检查原目标势场，再在容差区域中选最近的有限势场位置。自研实现沿用这个选择原则，
在 A* 搜索完起点可达区域后选后备终点；官方枚举矩形区域，本项目限制欧氏半径，
且仍以格中心为终点。两条路径的末点都使用实际选择的终点，并保留用户指定的朝向；
控制器以这个实际终点判断到达。节点日志记录原目标、替代终点和偏移距离。

`path_follower.yaml` 的线速度上限从 `0.10` 提高到 **0.15 m/s**，
角速度上限从 `0.40` 提高到 **0.55 rad/s**。转弯和接近目标时仍按现有控制逻辑减速，
地图、传感器或 TF 失效及碰撞风险时仍停车。配置在节点启动时加载，修改后需重启。
跟踪器先限制候选速度，再在两份原始碰撞几何中预测实际运动。
通常取最多 1 秒且不超过前视距离；反应行程与制动距离可要求更长预测。
每段不超过 0.05 s，并补偿圆弧到弦的偏离。名义速度被拒绝时检查少量减速、
调整曲率和原地转向候选，按进展、朝向、额外余量及速度变化评分。
安全替代运动状态为 `avoiding_obstacle`，同一轮替代最多 3 s 并请求重规划；
没有安全候选则锁存失败候选、持续零速重查，受阻期限不会被低速重新起步清掉。
`/mini_nav/control_diagnostics` 记录被检查候选、选中速度、首个冲突对象/距离及时间戳。
不默认执行倒车或允许起始重叠的脱离运动。

## 从当前机器人位置规划

收到 `/goal_pose` 后，节点查询最新的 `map -> base_footprint` TF，将当时车位
转换为规划图栅格起点。`/initialpose` 只重置定位并清除旧路径；节点会等待
`map -> odom` 在此次初始位姿之后更新，避免立即使用缓存中的旧定位结果。
`base_frame_id` 和 `odom_frame_id` 可按实际 TF 树配置，默认分别为
`base_footprint` 与 `odom`。

`planning.max_pose_age` 默认 1.0 s；TF 缺失、时间戳偏离当前时间过大、
车位越界或实际圆盘位置/连接不安全时，节点清空旧路径并拒绝规划。
目标越界、朝向无效或目标容差内没有可达安全终点时也发布空路径。
原始八邻域格点路径发布在 `/mini_nav/raw_path`；安全简化及平滑结果发布在
`/mini_nav/global_path`。每条候选捷径逐格检查硬禁行区、未知格与角点，
并用原始图检查基础车体半径加 map 定位预算的连续圆盘扫掠；
生产入口对未知格面积始终执行硬安全检查。
只有积分软代价不增加的安全修改才被采用；最终复核失败回退原始安全路径。
搜索阶段已逐边检查连续碰撞；后处理仍复核整条路径，复核失败时清空两条路径。
中间点朝向路径切线，
末点保留目标朝向；起点使用经过连接检查的实际连续位置，末点仍为所选格中心。RViz 同时显示橙色原始路径和
青色最终路径。`/mini_nav/global_path` 仍只是规划结果，不是行驶指令。

没有机器人 TF 的独立 `start_astar_navigation.sh` 演示显式启用
`planning.use_initial_pose_as_start`，继续用 RViz `/initialpose` 手选演示起点。
实际定位入口不启用该参数，规划起点始终来自 TF。

当前全局规划只处理静态图；路径跟踪器使用局部图检查碰撞风险并停车，
并支持持续动态障碍的有界重规划。车体朝向对应的精确多边形碰撞尚未实现；
安全半径应按实际车体及定位误差重新标定。
