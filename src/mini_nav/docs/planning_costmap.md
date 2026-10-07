# 规划代价地图

`costmap_publisher_node` 从 `/map`（或 `map_file`）读取原始静态图，保留
`/mini_nav/map` 的原始占据信息，并另外发布 `/mini_nav/planning_costmap`。
自研 A* 使用后者规划；AMCL 仍使用原始 `/map`。收到新地图时会重新生成规划图。

## 代价规则

- 障碍格为 `254`；未知格保留 `255`，两者均不可通行。原始 `/map` 不会改写。
- 候选格中心到障碍格**方形面积**或地图外边界的最短距离不超过
  `robot_radius + safety_margin` 时为 `253`。开启 `inflate_around_unknown` 后，
  未知格也在这个距离内生成 `253`；关闭时未知格自身仍为 `255`，但不向外传播。
- 仅确定障碍格在硬半径外至 `inflation_radius` 生成
  `max(1, floor(252 * exp(-cost_scaling_factor * (distance - hard_radius))))`
  的软代价；多个来源取最大值，未知格本身始终保持 `255`。
- A* 走八邻域，直边长度为 `1`、斜边为 `√2`；步长乘以
  `1 + cost_travel_multiplier * cell_cost / 252`，启发式为八方向距离。
  斜走时两侧正交格必须可通行，`253` 以上始终禁行。每条搜索边还用原始图
  检查 `robot_radius + safety_margin` 圆盘连续扫掠，因此无须额外半格余量。
  这个扫掠检查与未知膨胀开关同步：关闭时仍禁止中心线穿过未知格，
  但不要求车体圆盘与相邻未知格保持硬安全距离。地图外边界始终检查。

这个静态膨胀思路对应本仓库 `reference/nav2_costmap_2d_jazzy` 的
`InflationLayer::computeCost`。Nav2 的 `inflate_around_unknown` 控制未知格是否
作为膨胀源，`inflate_unknown` 控制未知格是否接收软代价；关闭后仍允许硬代价
覆盖未知格。二者在官方 Jazzy 源码中默认关闭。本项目以开关控制未知格周围
的车体硬余量，地图边缘始终保留硬余量；未知格从不生成软代价。
Nav2 使用格中心距离与车体内切半径；本项目用占据格面积距离与外接圆加余量。
A* 软代价对应
`reference/nav2_smac_planner_jazzy` 的 `Node2D::getTraversalCost`。
实现位于自研 `mini_nav_core`，运行时不依赖参考包。

## 参数与显示

两套定位加 A* 的 launch 均加载
`mini_nav_bringup/config/planning_costmap.yaml`：默认机器人半径 0.24 m、
额外安全余量 0.02 m、膨胀半径 0.45 m、衰减系数 10.0、A* 软代价权重 2.0。
仿真入口实际生成的 `turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf` 中，
底盘碰撞盒为 0.265 × 0.265 m，中心相对
`base_footprint` 的水平偏移为 -0.064 m；最远角点半径为
`√((0.064 + 0.265/2)² + (0.265/2)²) = 0.2370 m`。TF 所用
`turtlebot3_gazebo/urdf/turtlebot3_waffle.urdf` 的
0.266 × 0.266 m 碰撞盒算得 0.2377 m；轮、脚轮和激光雷达碰撞体均在此
半径内，因此 `robot_radius` 向上取整为 0.24 m。额外 0.02 m 使硬禁行半径
为 0.26 m，比最远碰撞角点多约 0.022 m；该余量尚未用定位误差数据标定。
不应把官方示例配置中的 0.15 m 当成该偏置碰撞盒的外接圆。软代价从硬边界
延伸 0.19 m，约 3.8 个 0.05 m 栅格。
若用独立节点启动，可用同名 `planning.*` ROS 参数覆盖。`robot_radius`
和 `inflation_radius` 必须为正，`safety_margin` 和权重必须为非负，且
`inflation_radius >= robot_radius + safety_margin`。
`planning.inflate_around_unknown` 默认为 `false`。两套定位 launch 还提供同名
`inflate_around_unknown` 启动参数，同时控制全局规划图和局部图；例如：

```bash
ros2 launch mini_nav_bringup mini_localization_astar.launch.py inflate_around_unknown:=true
```

参数在启动时读取，修改后需重启对应节点；关闭未知膨胀不会将未知格变为可通行。

RViz 的 **Planning Costmap** 显示订阅 `/mini_nav/planning_costmap`。
独立 A* 与定位视图均默认显示它；定位视图默认关闭 `Local Costmap` 和
距离场三个图层，避免颜色叠加。发布给 RViz 的占据值与内部代价分开：
`253` 硬安全区显示为 `99`（青色），`254` 障碍显示为 `100`（紫色），
`255` 未知显示为 `-1`；`1..252` 软代价映射到 `1..98`。显示透明度默认
为 `0.3`，与官方示例的全局代价地图一致。起点落在硬安全区时规划
返回失败；目标不可达时按下述目标容差寻找安全终点。

## 不可达目标与速度

`planning.goal_tolerance` 默认 **0.5 m**，必须为有限非负数，设为 `0` 禁用。
先尝试原目标栅格；若它位于障碍、未知格、硬安全区，车体余量不足或没有路径，
则从起点实际可达的安全格中选取距离原目标最近的终点。距离以目标栅格中心为基准，
使用欧氏距离；等距离候选优先选到达代价较低者。所有搜索边仍检查硬安全区、
斜向穿角和连续车体扫掠，目标容差不会放宽这些条件。容差内无可达点时发布空路径。
原目标可达时始终使用原目标；地图外的目标仍拒绝规划。

该行为参考官方 Jazzy [NavFnPlanner::makePlan](https://github.com/ros-navigation/navigation2/blob/f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501/nav2_navfn_planner/src/navfn_planner.cpp#L274-L310)：
官方先检查原目标势场，再在容差区域中选最近的有限势场位置。自研实现沿用这个选择原则，
在 A* 搜索完起点可达区域后选后备终点；官方枚举矩形区域，本项目限制欧氏半径，
且仍以格中心为终点。两条路径的末点都使用实际选择的终点，并保留用户指定的朝向；
控制器以这个实际终点判断到达。节点日志记录原目标、替代终点和偏移距离。

`path_follower.yaml` 的线速度上限从 `0.10` 提高到 **0.15 m/s**，
角速度上限从 `0.40` 提高到 **0.55 rad/s**。转弯和接近目标时仍按现有控制逻辑减速，
地图、传感器或 TF 失效及碰撞风险时仍停车。配置在节点启动时加载，修改后需重启。
跟踪器先计算候选速度，再在全局/局部安全图中预测最多 1 秒的实际运动，
范围不超过前视目标距离，每段连续扫掠同时覆盖圆弧偏离；当前位置安全时允许原地转向。

## 从当前机器人位置规划

收到 `/goal_pose` 后，节点查询最新的 `map -> base_footprint` TF，将当时车位
转换为规划图栅格起点。`/initialpose` 只重置定位并清除旧路径；节点会等待
`map -> odom` 在此次初始位姿之后更新，避免立即使用缓存中的旧定位结果。
`base_frame_id` 和 `odom_frame_id` 可按实际 TF 树配置，默认分别为
`base_footprint` 与 `odom`。

`planning.max_pose_age` 默认 1.0 s；TF 缺失、时间戳偏离当前时间过大、
车位越界或落在代价 `>=253` 的硬安全区时，节点清空旧路径并拒绝规划。
目标越界、朝向无效或目标容差内没有可达安全终点时也发布空路径。
原始八邻域格点路径发布在 `/mini_nav/raw_path`；安全简化及平滑结果发布在
`/mini_nav/global_path`。每条候选捷径逐格检查硬禁行区、未知格与角点，
并用原始图检查半径为 `robot_radius + safety_margin` 的连续圆盘扫掠；
未知格是否参与扫掠检查由同一个开关控制。
只有积分软代价不增加的安全修改才被采用；最终复核失败回退原始安全路径。
搜索阶段已逐边检查连续碰撞；后处理仍复核整条路径，复核失败时清空两条路径。
中间点朝向路径切线，
末点保留目标朝向；起终点仍是格中心。RViz 同时显示橙色原始路径和
青色最终路径。`/mini_nav/global_path` 仍只是规划结果，不是行驶指令。

没有机器人 TF 的独立 `start_astar_navigation.sh` 演示显式启用
`planning.use_initial_pose_as_start`，继续用 RViz `/initialpose` 手选演示起点。
实际定位入口不启用该参数，规划起点始终来自 TF。

当前全局规划只处理静态图；路径跟踪器使用局部图检查碰撞风险并停车，
但尚不进行动态绕障重规划。车体朝向对应的精确多边形碰撞尚未实现；
安全半径应按实际车体及定位误差重新标定。
