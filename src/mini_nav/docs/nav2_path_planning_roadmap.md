# mini_nav 路径规划路线报告：从教学型 A* 到完整 Nav2

> 状态：规划报告（基于当前工作区与 ROS 2 Jazzy/Nav2 官方资料的调查整理）
> 适用平台：ROS 2 Jazzy、Gazebo、TurtleBot3 Waffle
> 范围：全局路径规划、代价地图、路径跟踪、行为编排与安全闭环
> 文档类型：路线规划与验收基线；本报告不替代具体实现变更记录。

## 1. 目标与结论

本项目当前的 A* 适合学习“栅格地图上如何搜索一条路”，但还不是可执行的机器人导航系统。完整可用的 Nav2 路线应按职责逐步补齐：先建立官方 Nav2 基线，再统一代价地图语义，随后升级自研 A* 和路径姿态，最后通过 `nav2_core::GlobalPlanner` 插件接入 Planner Server，并补齐 Controller Server、行为树、恢复行为和速度安全链路。

推荐的默认终点是由 BT Navigator 编排官方服务器，使用 SmacPlanner2D 作为全局规划基线、Controller Server 负责路径跟踪，并将代价地图、恢复行为和速度安全链路分别纳入对应组件。

自研 A* 应保留为教学和 A/B 对照实现；只有当它在代价、碰撞、姿态、取消和生命周期边界上满足插件验收后，才将其作为 Nav2 的可选全局规划器。对当前近似圆形、可原地旋转的差速底盘，先采用 **SmacPlanner2D + 正确的全局/局部代价地图 + 控制器**，不要一开始就跳到 Hybrid-A* 或 State Lattice。

Nav2 的控制关系应理解为“BT Navigator 编排各服务器”，而不是一条固定的 Planner → Smoother → Controller 下游流水线：

```text
map_server + AMCL ──> map / TF / global costmap
/scan + /odom ───────> TF / local costmap

                         ┌────────────────────────────────┐
                         │ BT Navigator                    │
                         │ NavigateToPose / replanning     │
                         └──────────┬───────────┬───────────┘
                                    │           │
                         ComputePathToPose   FollowPath
                                    │           │
                             Planner Server  Controller Server ──> Velocity Smoother
                                    │           │                          │
                           Global Costmap  Local Costmap       Collision Monitor（可选）
                                    │           │                          │
                         Smoother Server（可选）                ────────> /cmd_vel
                         （BT 流程中的路径动作）

Behavior Server（恢复动作，由 BT 调用）
Lifecycle Manager（独立管理所有 Nav2 服务器的生命周期）
```

图中 Smoother Server 和 Behavior Server 都是由 BT 流程调用的动作服务器；它们不是必须串联在 Planner 和 Controller 之间的固定下游节点。Controller 是否能对动态障碍局部规避，取决于所选控制器及其代价地图配置。

## 2. 当前 `mini_nav` 现状

以下事实来自 `src/mini_nav/docs/` 中的项目状态、架构和官方路径规划调研文档；“现状”与“建议”分开记录，避免把未来方案误当作已经存在的代码。

### 2.1 已有运行基础

- 目标平台为 ROS 2 Jazzy，仿真机器人为 TurtleBot3 Waffle。
- 已复用 Gazebo、地图、机器人模型以及 `/scan`、`/odom`、TF 和速度接口。
- 项目同时保留官方 AMCL 对照入口和自研 AMCL；当前定位链路约定为 `map -> odom -> base_footprint`。
- AMCL 负责 `map -> odom`，里程计或仿真负责 `odom -> base_*`；不得再添加静态 `map -> odom`，否则会产生 TF 冲突。
- `src/mini_nav/mini_nav_core/` 保持 ROS 无关的算法核心，`src/mini_nav/mini_nav_nodes/` 负责 ROS 适配，`src/mini_nav/mini_nav_bringup/` 负责启动、参数和仿真编排。

### 2.2 当前 A* 与 ROS 适配边界

- `src/mini_nav/mini_nav_core/src/navigator/astar_navigator.cpp` 使用四邻域、Manhattan 启发式和均匀步长 `g += 1`。
- 当前只将 lethal 栅格视为不可通行，没有 Nav2 连续代价场、膨胀代价或机器人 footprint 语义。
- `CostmapPublisherNode` 将地图转换为项目自己的 `Costmap2D`，接收 `/initialpose` 作为起点，接收 `/goal_pose` 后执行一次性规划。
- 规划结果发布到 `/mini_nav/global_path`；路径点的朝向尚未按路径切线计算。
- 当前 `OccupancyGrid` 被简化为 free/unknown/lethal 三类，而 Nav2 运行时需要可表达障碍、膨胀和未知策略的连续代价。
- 当前官方 launch 是“定位 + 自研 A*”对照入口，不是完整 Nav2 栈；`/mini_nav/global_path` 也不是可直接交给控制器执行的完整导航轨迹。

## 3. 主要缺口

### 3.1 环境表示

1. 缺少由 `nav2_costmap_2d` 管理的全局与局部代价地图。
2. 尚未配置 `StaticLayer`、`ObstacleLayer`、`InflationLayer` 的组合及其更新、标记、清除和未知空间策略。
3. 尚未统一 `robot_radius` 或多边形 footprint、碰撞阈值、膨胀半径和代价衰减参数。
4. 自研 `Costmap2D` 的教学语义不能直接替代 Nav2 的运行时代价地图。

### 3.2 规划服务与插件边界

1. 当前 `/goal_pose` 回调是一次性直接调用，不具备 Planner Server 的 action、`planner_id`、取消检查和失败状态语义。
2. 自研规划器尚未实现 `nav2_core::GlobalPlanner` 的生命周期方法。
3. 还没有通过 pluginlib 注册、加载和配置多个规划器的 A/B 接口。

### 3.3 路径质量与可执行性

1. 四邻域和均匀步长容易生成直角折线；对角移动、对角穿角禁止规则和代价敏感搜索尚未具备。
2. 尚未明确 unknown 的规划策略，也没有对起点/终点、越界和 lethal 栅格做完整策略化处理。
3. 路径没有稳定的切线朝向、目标姿态保持、平滑和碰撞复检。

### 3.4 执行、编排和安全

1. 没有 Controller Server/`FollowPath`/goal checker/progress checker 的完整执行链。
2. 动态障碍尚未通过局部代价地图和控制器形成闭环；一次全局规划不能解决持续阻塞。
3. 没有 BT Navigator、Behavior Server、重规划和恢复行为。
4. 速度平滑、紧急停车和可选 Collision Monitor 尚未纳入验收。
5. 当前记录的 `/cmd_vel` 类型为 `geometry_msgs/msg/TwistStamped`，必须确认未来控制器输出与 Gazebo bridge 的实际接口完全匹配。

## 4. 分阶段路线

每个阶段都应先完成验收，再进入下一阶段。未来文件均显式标记“建议新增（未来）”，不表示当前已经存在。

### Phase 0：建立官方 Nav2 Golden Baseline

**目标**：在不删除或重写自研 A* 的前提下，跑通官方 `NavigateToPose` 全链路，得到可重复的基准。

**官方组件**：`map_server`、AMCL、Planner Server、`SmacPlanner2D`、`NavFn`（第二基线）、全局/局部 costmap、Controller Server（建议先用 RPP）、Smoother Server、BT Navigator、Behavior Server 和 Lifecycle Manager。

**项目影响**：

- `src/mini_nav/mini_nav_bringup/config/nav2_waffle.yaml`（建议新增，未来）
- `src/mini_nav/mini_nav_bringup/launch/official_nav2.launch.py`（建议新增，未来）
- 仅增加官方对照入口，不改现有 self-A* 入口和核心算法。

**验收**：Lifecycle 节点进入 active；地图、AMCL、TF、全局/局部 costmap 均可观察；通过 `NavigateToPose` 让 Waffle 到达目标并安全停止。

**本阶段不做**：不把自研 A* 强行塞进 Planner Server，不同时启动两套会发布同名 TF 的定位节点，不以一次 RViz 路径显示替代完整导航验收。

### Phase 1：统一 Nav2 代价地图语义

**目标**：先让规划器看到正确的环境和机器人安全边界，再比较算法优劣。

**实现重点**：

- 全局地图使用 `StaticLayer`；传感器观测使用 `ObstacleLayer`；障碍周围使用 `InflationLayer`。
- 明确 `robot_radius` 或多边形 footprint、`inflation_radius`、`cost_scaling_factor`、lethal threshold 和 unknown policy。
- 分离全局 costmap 与局部 costmap 的更新频率、范围、滚动窗口和传感器观测范围。

**项目边界**：自研 `src/mini_nav/mini_nav_core/src/map/costmap_2d.cpp` 继续作为教学模块和单元测试对象；官方运行入口统一使用 `nav2_costmap_2d`。

**验收**：在 RViz 和话题中可确认静态障碍、传感器障碍、膨胀区域、未知区域策略与机器人 footprint 一致；规划路径不穿越 footprint 碰撞区。

### Phase 2：升级 ROS 无关的自研二维 A*

**目标**：让自研算法具备与 SmacPlanner2D 可公平比较的二维搜索能力。

**建议修改文件**：

- `src/mini_nav/mini_nav_core/include/mini_nav_core/navigator/astar_navigator.hpp`
- `src/mini_nav/mini_nav_core/src/navigator/astar_navigator.cpp`
- `src/mini_nav/mini_nav_core/test/test_astar_planner.cpp`

**建议能力**：

1. 可配置 4/8 邻域；对角步长使用 `sqrt(2)`。
2. 禁止对角穿过两个相邻 lethal 单元（corner cutting）。
3. 让 `g` 使用栅格代价和距离代价，区分穿越高代价区域与安全区域。
4. 采用与邻域匹配的 octile 或 Euclidean 启发式，并保持启发式不高估。
5. 明确 unknown policy、起点/终点非法、无路和 `start == goal` 行为。
6. 保持核心不依赖 `rclcpp`、TF、action、参数服务器或发布器。

**验收**：核心单元测试覆盖 4/8 邻域、穿角阻挡、代价敏感绕行、未知策略、无路、越界、起终点非法、起点等于终点和启发式基本性质。

### Phase 3：路径姿态、平滑与碰撞复检

**目标**：把“栅格点序列”变成具有稳定朝向且经过安全复核的 `nav_msgs/Path`。

**建议设计**：

- `src/mini_nav/mini_nav_core/include/mini_nav_core/navigator/path_utils.hpp`（建议新增，未来）
- `src/mini_nav/mini_nav_core/src/navigator/path_utils.cpp`（建议新增，未来）
- 根据相邻点切线计算 yaw；终点保留用户目标姿态，起点保留当前姿态或按首段切线处理。
- 记录路径长度、航向变化、最小 clearance 和 waypoint 数，作为后续 A/B 指标。
- 优先使用官方 Smoother Server 做对照；自研平滑器输出后必须重新进行 footprint 碰撞检查，失败时回退到未平滑路径或报告失败。

**验收**：路径 `header.frame_id`、时间戳、姿态序列、目标朝向和碰撞结果稳定；平滑不会把路径推入障碍物。

### Phase 4：以 GlobalPlanner 插件接入 Planner Server

**目标**：让自研 A* 使用官方 `ComputePathToPose` 和 Planner Server 管理边界，而不是继续维护一套平行导航服务器。

**建议新增包**：`src/mini_nav/mini_nav_nav2_plugins/`（建议新增，未来）。

适配器应实现 `nav2_core::GlobalPlanner` 的 `configure()`、`activate()`、`deactivate()`、`cleanup()` 和 `createPlan()`，只负责：

- 读取插件参数、保存 TF 和 Nav2 costmap wrapper；
- 将 Nav2 `Costmap2D` 转换为核心所需的只读视图；
- 处理起点/终点坐标系、取消检查和时间戳；
- 将核心结果转成带姿态的 `nav_msgs::msg::Path`；
- 统一错误和无路返回。

**A/B 顺序**：在同一个 Planner Server 中依次配置 NavFn、SmacPlanner2D 和 `MiniNavGlobalPlanner`。这样可以保持 action、生命周期、costmap 和控制器不变，只比较规划器本身。

**本阶段不做**：适配器不自行创建 Planner Server、action server、Controller Server 或 Lifecycle Manager；`mini_nav_core` 仍不依赖 ROS。

### Phase 5：补齐控制器、动态障碍与重规划

**目标**：从“有一条全局路径”变成“机器人能沿路径运动，并对短时障碍作局部反应”。

**实现重点**：

- 配置局部 costmap 的 `ObstacleLayer` 和滚动窗口。
- 通过 Controller Server/`FollowPath` 让控制器根据局部代价和路径生成速度。
- 确认 `/cmd_vel` 的消息类型、QoS、frame 和 Gazebo bridge 接口。
- 由 BT Navigator 定期或按阻塞条件重新调用全局规划；局部代价地图先把障碍暴露给控制器，控制器可根据所选插件能力减速、停止或局部规避，长期阻塞才触发全局重规划或恢复行为。

**验收**：插入动态障碍后，可观察 scan → obstacle marking → controller reaction → 必要时 replanning 或恢复；具体反应允许是减速、停止、局部规避或重规划中的一种或多种，但机器人不得继续向已占用区域输出不安全速度。

### Phase 6：恢复行为与速度安全闭环

**目标**：满足“完全可用”的工程条件，而不只是实验室中一次成功。

**官方组件**：Behavior Server、goal/progress checker、恢复行为树、Velocity Smoother，以及按需要启用 Collision Monitor。

**验收**：

- 无进展、目标不可达、局部代价图阻塞时能进入明确的恢复或失败状态。
- 速度限制、加减速度平滑和急停行为可观测、可测试。
- 导航完成、取消、超时和安全停止均能回到可再次接收目标的稳定状态。

### Phase 7：按真实运动学需求选择高级规划器

只有当需求明确要求以下能力时才进入本阶段：最小转弯半径、SE(2) 朝向约束、倒车、非圆形 footprint 或自定义控制集。

- **SmacPlannerHybrid**：用于带朝向和转弯半径的 Hybrid-A*，适合需要运动学可行路径的场景。
- **SmacPlannerLattice**：用于预生成控制集、非圆形底盘或自定义运动原语。
- **Theta***：可作为 any-angle 学习和路径锯齿对照，不应被当作完整差速运动学规划器。

默认仍保持 `SmacPlanner2D + 正确代价地图 + 控制器`，除非真实验收指标证明二维粒子近似不足。

## 5. ROS 2/Nav2 组件映射

| 当前/教学职责 | 完整 Nav2 目标组件 | 迁移说明 |
|---|---|---|
| 静态地图 | `nav2_map_server` | 继续复用现有地图资源，统一由官方生命周期节点提供 |
| 定位与 `map -> odom` | `nav2_amcl` 或已验证的自研 AMCL | 同一运行入口只允许一个节点拥有该 TF |
| 自研 `Costmap2D` | `nav2_costmap_2d` | 自研版本保留教学用途，官方运行时使用标准 costmap |
| 静态障碍 | `StaticLayer` | 读取 map_server/SLAM 地图 |
| 动态传感器障碍 | `ObstacleLayer` | 处理 `/scan` 的 marking/clearing |
| 安全距离代价 | `InflationLayer` | 由 footprint、半径和膨胀参数决定 |
| 直接 A* 调用 | Planner Server | 统一 action、planner_id、取消和错误状态 |
| 自研 planner 接缝 | `nav2_core::GlobalPlanner` | 只做薄 ROS 适配器，核心保持 ROS 无关 |
| 官方二维基线 | `SmacPlanner2D` | 默认首选，适合当前差速 Waffle 的二维近似 |
| 官方第二基线 | `NavFn` | 用于独立验证 costmap、Planner Server 和 action 链路 |
| any-angle 对照 | `ThetaStarPlanner` | 只用于直线性/锯齿对照，不解决完整运动学 |
| 路径后处理 | Smoother Server | 平滑后必须进行碰撞复检 |
| 路径执行 | Controller Server + `FollowPath` | 将路径转为速度并处理局部避障 |
| 导航入口与重规划 | BT Navigator | 管理 NavigateToPose、重规划和行为树流程 |
| 恢复与行为动作 | Behavior Server | 清图、旋转、等待、后退等恢复能力 |
| 生命周期编排 | Lifecycle Manager | 确保节点按顺序 configure/activate |
| 速度整形 | Velocity Smoother | 限制速度、加速度和输出抖动 |
| 可选硬安全层 | Collision Monitor | 在控制器之外提供速度限制或急停边界 |

## 6. 文件与模块演进

| 阶段 | 现有文件/目录 | 建议新增或演进 | 约束 |
|---|---|---|---|
| 0 | `src/mini_nav/mini_nav_bringup/` | `config/nav2_waffle.yaml`、`launch/official_nav2.launch.py`（建议新增，未来） | 仅增加官方对照入口 |
| 1 | `mini_nav_core/src/map/costmap_2d.*` | Nav2 costmap 参数（随 Phase 0 配置，建议新增，未来） | 教学 costmap 不冒充 Nav2 runtime |
| 2 | `mini_nav_core/include/mini_nav_core/navigator/astar_navigator.hpp`、`src/.../astar_navigator.cpp` | 4/8 邻域、代价、未知策略和穿角规则 | 保持 ROS-free |
| 2 | `mini_nav_core/test/test_astar_planner.cpp` | 扩展边界、代价和连通性测试 | 先单元测试再接 ROS |
| 3 | `mini_nav_core/navigator/` | `path_utils.*`（建议新增，未来） | 平滑后碰撞复检 |
| 4 | `src/mini_nav/` | `mini_nav_nav2_plugins/`（建议新增，未来） | 只实现 `GlobalPlanner` 适配器 |
| 5–6 | `mini_nav_bringup/` | Controller/BT/Behavior/安全参数与 launch（建议新增，未来） | 确认 `/cmd_vel` 类型和 QoS |

## 7. 关键风险与规避

1. **TF 重复发布**：官方 AMCL、自研 AMCL 和静态 TF 不能同时拥有同一条 `map -> odom`；每个 launch 必须明确唯一 owner。
2. **两套 costmap 语义混用**：不要把自研三值 `Costmap2D` 直接当作 Nav2 连续代价地图；插件适配时使用 Nav2 的真实 costmap wrapper。
3. **planner/controller 职责混淆**：Planner 只返回路径，Controller 才生成速度；动态障碍、重规划和恢复是多个组件的协作结果。
4. **footprint/膨胀参数失真**：半径过小会贴墙或碰撞，过大会导致无路；参数必须结合 Waffle 实际几何和实测 clearance 调整。
5. **对角穿角和未知空间**：8 邻域若不禁止穿角会产生几何上不可行的路径；unknown policy 不明确会导致地图边界行为不稳定。
6. **平滑后碰撞**：任何路径平滑都可能改变障碍间隙，必须在平滑后重新检查 footprint 碰撞。
7. **插件生命周期与 pluginlib**：configure/activate/deactivate/cleanup、XML 导出、命名空间和参数拼写均需独立测试。
8. **速度接口不一致**：当前记录的 `/cmd_vel` 为 `TwistStamped`，控制器、bridge 和底盘驱动必须端到端验证。
9. **脏工作区污染**：当前分支已有未提交修改；实现时只应触碰任务声明的文件，不要清理或覆盖无关变更。

## 8. 验证与测试矩阵

### 8.1 核心 A* 单元测试

- 起点/终点越界、lethal、unknown 和无路。
- `start == goal`、单格通路和最小地图。
- 4 邻域、8 邻域、对角代价和对角穿角阻挡。
- 高代价直线路径与低代价绕行路径的选择。
- 不同 unknown policy 下的连通性。
- 路径首尾、相邻性、无重复点、代价累计和启发式基本性质。

### 8.2 GlobalPlanner 插件测试（Phase 4 起）

- 生命周期 configure/activate/deactivate/cleanup。
- 正常规划、无路、非法起终点和目标坐标系错误。
- 取消检查能及时返回；输出 `frame_id`、时间戳、姿态和目标朝向正确。
- NavFn、SmacPlanner2D、自研插件在相同地图/起终点下可重复运行。

### 8.3 Launch 与集成测试

- `ros2 launch ... --show-args`、包 share 路径、参数文件、地图路径和 `use_sim_time`。
- Lifecycle 节点最终为 active；检查 `/map`、`/scan`、`/odom`、TF、全局/局部 costmap、Planner/Controller action。
- 发送 `NavigateToPose`，验证规划、跟踪、到达、取消、超时和安全停止。
- 动态障碍序列：scan → obstacle marking → 控制器反应 → 重规划/恢复 → 到达或安全停止。

### 8.4 A/B 指标

在相同地图、起终点、footprint、CPU 和 costmap 参数下记录：规划时间、路径长度、waypoint 数、航向变化次数、最小 clearance、导航完成时间、重规划次数、振荡次数和失败原因。不能只用“RViz 看起来更直”作为结论。

## 9. 最终验收清单

- [ ] 官方 Nav2 Golden Baseline 可用，Waffle 能通过 `NavigateToPose` 到达并停止。
- [ ] 全局/局部 costmap 的 Static、Obstacle、Inflation 层和 footprint 参数经过实测。
- [ ] 自研 A* 具备 4/8 邻域、代价敏感、禁止穿角、unknown policy 和完整边界测试。
- [ ] 路径含正确 frame、时间戳、姿态和目标朝向，平滑后通过碰撞复检。
- [ ] 自研 planner 通过 `nav2_core::GlobalPlanner` 接入 Planner Server，可与 NavFn/SmacPlanner2D A/B。
- [ ] Controller Server 能跟踪路径，动态障碍会触发局部反应和必要的全局重规划。
- [ ] BT Navigator、Behavior Server、goal/progress checker、Lifecycle Manager 和恢复路径可验证。
- [ ] Velocity Smoother/Collision Monitor（按需求）能限制速度并在异常时安全停止。
- [ ] TF owner、`/cmd_vel` 类型/QoS、参数、pluginlib 和脏工作区边界均有记录。
- [ ] 以测试矩阵和 A/B 指标形成可重复的回归结果，而不是仅依赖人工观察。

## 10. 事实边界与实施原则

- “当前”仅指本报告调查时工作区中已经存在并由项目文档描述的内容；带“建议新增（未来）”的路径不是现有文件。
- Nav2 官方组件是目标运行时与对照基线，不要求删除 `mini_nav` 的学习型核心；自研核心和官方插件适配器应保持清晰分层。
- 从 A* 到完整导航不是一次替换规划器的工作，而是环境表示、规划服务、路径质量、控制执行、行为编排和安全闭环的连续迁移。
- 每一阶段都以可观察的接口、自动化测试和可复现指标验收；未满足验收时不进入下一阶段。
