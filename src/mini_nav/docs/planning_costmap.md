# 规划代价地图

`costmap_publisher_node` 从 `/map`（或 `map_file`）读取原始静态图，保留
`/mini_nav/map` 的原始占据信息，并另外发布 `/mini_nav/planning_costmap`。
自研 A* 使用后者规划；AMCL 仍使用原始 `/map`。收到新地图时会重新生成规划图。

## 代价规则

- 障碍格为 `254`；未知格保留 `255`，两者均不可通行。
- 障碍中心到候选格中心的欧氏距离不超过 `robot_radius + safety_margin`
  时，候选格为 `253`，A* 也将其视为不可通行。
- 到障碍的距离在硬安全半径与 `inflation_radius` 之间时，代价为
  `max(1, floor(252 * exp(-cost_scaling_factor * (distance - hard_radius))))`。
  多个障碍的代价取最大值，原始图不会被改写。
- A* 的四邻域单步代价为
  `1 + cost_travel_multiplier * cell_cost / 252`，启发式仍为曼哈顿距离。
  因此最短距离与障碍余量可以权衡；`253` 以上始终禁行。

这个静态膨胀思路对应本仓库 `reference/nav2_costmap_2d_jazzy` 的
`InflationLayer::computeCost`，A* 软代价对应
`reference/nav2_smac_planner_jazzy` 的 `Node2D::getTraversalCost`。
实现位于自研 `mini_nav_core`，运行时不依赖参考包。

## 参数与显示

两套定位加 A* 的 launch 均加载
`mini_nav_bringup/config/planning_costmap.yaml`：默认机器人半径 0.24 m、
额外安全余量 0.05 m、膨胀半径 0.55 m、衰减系数 5.0、A* 软代价权重 2.0。
0.24 m 依据当前 Waffle URDF 底盘碰撞盒角点约 0.238 m 的外接距离向上取整。
若用独立节点启动，可用同名 `planning.*` ROS 参数覆盖。`robot_radius`
和 `inflation_radius` 必须为正，`safety_margin` 和权重必须为非负，且
`inflation_radius >= robot_radius + safety_margin`。

RViz 的 **Planning Costmap** 显示订阅 `/mini_nav/planning_costmap`。
独立 A* 视图默认显示；定位视图为避免与观测距离色块重叠，默认关闭，
需要时在 Displays 中打开。白色/高代价区域是不可通行的安全区，
周边颜色表示软代价。起点或目标落在硬安全区时规划返回失败。

当前只处理静态图中的确定障碍物。未知区域保持不可通行，但不向已知区域
膨胀；动态障碍、车体朝向对应的精确多边形碰撞、路径平滑和局部控制不在
这张规划图内。安全半径应按实际车体及定位误差重新标定。
