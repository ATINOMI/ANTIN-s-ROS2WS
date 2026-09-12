# ROS 2 Jazzy / Nav2 官方路径规划方案调研

最后更新：2026-08-31

## 0. 调研范围与阅读约定

本文服务于 /home/a/ros2_ws/src/mini_nav 自研导航项目。当前项目使用的 Gazebo、地图和机器人模型仍来自官方示例；本文只研究如何把官方路径规划方案作为对照、升级方向或后续可接入的模块，不把官方仿真资产误认为 mini_nav 自研内容。

资料范围限定为 Nav2 Jazzy 官方文档、Navigation2 官方 GitHub Jazzy 源码/README，以及 ROS 2 Jazzy 官方接口/API 文档。文中用“官方事实”表示官方资料直接说明的行为；用“本项目建议”或“本项目推断”表示结合当前 mini_nav 代码后的工程判断。

本文中的“规划器”默认指全局规划器（global planner）：它根据起点、终点和全局代价地图产生全局路径；“控制器”负责沿路径生成速度指令，并处理局部可执行性和动态避障。不要把更换全局规划器和实现路径跟踪混为一件事。Nav2 对规划、控制、平滑和代价地图的职责划分见[Navigation Servers 官方概念说明](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/)。

## 1. 结论摘要

### 1.1 直接结论

- **官方事实：** Nav2 的 Planner Server 管理一个或多个全局规划器插件，收到规划请求后根据 planner_id 选择插件，并把全局代价地图提供给插件。插件通过 nav2_core::GlobalPlanner 接口接入。详见[Planner Server 配置文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/)和[编写全局规划器插件教程](https://docs.nav2.org/jazzy/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/)。
- **官方事实：** Jazzy 官方提供 NavFn、SmacPlanner2D、SmacPlannerHybrid、SmacPlannerLattice 和 ThetaStarPlanner 等规划器；Theta* 在 Jazzy 仍然受支持。Jazzy 的[导航插件注册表](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)和[Jazzy 迁移指南中的 Planner Server 插件列表](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/iron/Iron/)均可核对这一点。
- **本项目建议：** 当前 mini_nav 是四邻域、均匀步长代价、无膨胀、无路径后处理的教学型 A*。对当前差速、近似圆形机器人，第一阶段优先以 **SmacPlanner2D** 作为官方基线；它仍是二维 A*，但补足了 4/8 邻域、代价敏感搜索、降采样和简单平滑等关键能力。依据见[Smac 2D 配置文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/)。
- **本项目建议：** 如果当前首要问题是折线路径的锯齿，而不是最小转弯半径，Theta* 可以作为独立的学习和对照分支；它用可见性检查生成 any-angle 路径，但仍假设二维全向粒子，不能单独解决差速车的运动学约束。依据见[Theta* 配置文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/)和[Theta* Jazzy 官方 README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_theta_star_planner)。
- **本项目建议：** 只有当需求明确包含最小转弯半径、非圆形 footprint、倒车或严格的 SE(2) 可执行性时，再评估 SmacPlannerHybrid 或 SmacPlannerLattice。它们不是单纯“让路径看起来更顺”的替代品，而是把朝向、运动原语、曲率或控制集纳入搜索状态。依据见[Smac 官方 README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smac_planner)。
- **本项目推断：** 动态障碍物问题不能靠单独把 A* 换成另一个全局规划器解决。需要把传感器观测写入 ObstacleLayer，设置合适的 InflationLayer，并由局部代价地图、控制器和重规划策略共同工作。相关职责依据见[Costmap 2D 官方文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)和[环境表示概念说明](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/environmental_representation/)。

### 1.2 推荐升级顺序

1. 先运行官方 Planner Server + 全局代价地图，使用 NavFn 和 SmacPlanner2D 做可重复的基线对比。
2. 补齐 mini_nav 的代价语义：未知、致命障碍、膨胀代价、机器人半径/footprint 和碰撞检查。
3. 把当前 A* 升级为代价敏感的 8 邻域二维搜索，加入禁止对角穿角和路径朝向生成。
4. 把平滑器作为独立阶段，并在平滑后重新做碰撞检查。
5. 如果需要继续维护自研规划器，再把 ROS 适配层改造成 nav2_core::GlobalPlanner 插件；算法核心仍保持 ROS 无关。
6. 之后再接入动态障碍物层、局部代价地图、控制器和重规划。
7. 最后根据最小转弯半径、倒车和非圆形 footprint 等真实需求决定是否进入 Hybrid-A* 或 State Lattice。

这个顺序是**本项目建议**，不是 Nav2 官方强制迁移路线；每一步的官方依据和推断边界在第 6 节展开。

## 2. Nav2 的规划架构与插件接口

### 2.1 Planner Server 的职责

**官方事实：** Planner Server 是全局规划器的 ROS 服务器。它维护规划器插件集合，接受规划请求中的目标和规划器名称，调用指定插件计算路径，并管理规划器使用的全局代价地图。一个 Planner Server 可以配置多个插件，每个插件通过自己的命名空间配置。

官方配置的核心形式如下，GridBased 只是一个用户定义的插件别名：

~~~yaml
planner_server:
  ros__parameters:
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_navfn_planner::NavfnPlanner"
~~~

以上插件列表、插件类型映射以及 Planner Server 参数来自[Configuring Planner Server（Jazzy）](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/)；Planner Server 的 Jazzy 源码位于[Navigation2/nav2_planner](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_planner)。

可以把官方运行链路抽象为：

~~~text
ComputePathToPose action
        |
        v
Planner Server
        |
        +--> 根据 planner_id 选择 GlobalPlanner 插件
        |          |
        |          +--> 读取全局 Costmap2D
        |          +--> 返回 nav_msgs/Path
        |
        +--> 规划失败状态、规划耗时等 action 结果

nav_msgs/Path
        |
        v
Controller Server / FollowPath
        |
        +--> 读取局部代价地图
        +--> 计算速度指令
        +--> 跟踪路径并处理局部动态障碍
~~~

上面“控制器负责跟踪和局部控制”的职责划分见[Navigation Servers 官方说明](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/)，规划动作和控制动作的字段见[ComputePathToPose Jazzy API](https://api.nav2.org/actions/jazzy/computepathtopose.html)和[FollowPath Jazzy API](https://api.nav2.org/actions/jazzy/followpath.html)。

### 2.2 nav2_core::GlobalPlanner 接口

**官方事实：** Jazzy 的自定义全局规划器应继承 nav2_core::GlobalPlanner，并实现生命周期相关方法和规划方法。官方插件教程列出的核心方法是：

- configure(...)：接收父节点、插件实例名称、TF buffer 和 costmap wrapper，读取参数并保存运行时依赖。
- activate()：开始使用 ROS 通信对象或发布器。
- deactivate()：暂停运行时通信。
- cleanup()：释放资源。
- createPlan(...)：接收起点、终点和取消检查器，返回 nav_msgs::msg::Path。

接口的 ROS 2 Jazzy API 索引见[nav2_core::GlobalPlanner](https://docs.ros.org/en/jazzy/p/nav2_core/generated/file_include_nav2_core_global_planner.hpp.html)；包含完整步骤、插件 XML 和 PLUGINLIB_EXPORT_CLASS 用法的官方教程见[Writing a New Planner Plugin](https://docs.nav2.org/jazzy/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/)。

**本项目建议：** 如果将来要把自研 A* 接到官方 Planner Server，建议把 AStarPlanner 的 ROS 无关算法接口和一个很薄的 GlobalPlanner 适配器分开。不要让算法核心直接依赖 action、参数服务器、TF 或发布器；这些属于节点/插件边界。这样既能和 mini_nav 现有学习代码对比，也能通过官方 Planner Server 使用同一个 ComputePathToPose 接口。

### 2.3 路径消息和动作接口

**官方事实：** nav_msgs/Path 是由一组带时间戳和坐标系信息的 geometry_msgs/PoseStamped 组成的路径消息，字段为：

~~~text
std_msgs/Header header
geometry_msgs/PoseStamped[] poses
~~~

字段定义见[ROS 2 Jazzy nav_msgs README](https://docs.ros.org/en/jazzy/p/nav_msgs/README.html)。

| 接口 | 作用 | 关键字段 |
|---|---|---|
| nav2_msgs/action/ComputePathToPose | 请求从起点到目标点的全局路径 | goal、可选 start、planner_id、use_start；结果为 nav_msgs/Path、规划耗时和错误信息 |
| nav2_msgs/action/ComputePathThroughPoses | 请求经过多个目标位姿的路径 | 多个目标位姿、规划器选择和路径结果 |
| nav2_msgs/action/FollowPath | 让控制器跟踪一条已有路径 | nav_msgs/Path、controller_id、goal/progress checker ID；反馈包括到目标距离和速度 |
| nav2_msgs/action/SmoothPath | 调用平滑器处理一条路径 | 输入路径、smoother_id、最大平滑时长、是否碰撞检查；结果为平滑后路径和完成状态 |

动作定义和字段应以[Jazzy actions API 索引](https://api.nav2.org/jazzy/actions/)及其[ComputePathToPose](https://api.nav2.org/actions/jazzy/computepathtopose.html)、[FollowPath](https://api.nav2.org/actions/jazzy/followpath.html)、[SmoothPath](https://api.nav2.org/actions/jazzy/smoothpath.html)页面为准。

**本项目推断：** 当前 mini_nav 的 /goal_pose 回调直接触发一次规划、再发布自定义 /mini_nav/global_path，适合演示 A*，但还不是 Planner Server 的 action/plugin 边界。后续若要和 Nav2 控制器或工具链对接，至少需要让规划结果稳定地表达为带 frame_id、时间戳、位姿序列和朝向的 nav_msgs/Path。

## 3. 代价地图、障碍层与 InflationLayer

### 3.1 代价地图是规划器的环境输入

**官方事实：** Nav2 Costmap 2D 是规则二维网格，单元格表达未知、空闲、占用和膨胀后的代价；全局规划器用全局代价地图搜索路径，控制器用局部代价地图评估和生成局部控制。官方说明见[环境表示](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/environmental_representation/)和[Costmap 2D 配置指南](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)。

全局代价地图常见的插件组合是：

~~~text
StaticLayer      <- map_server 或 SLAM 提供的静态地图
ObstacleLayer    <- 激光、深度等传感器的标记/清除观测
InflationLayer   <- 障碍物周围的安全代价场
        |
        v
Planner Server / Controller Server 使用的 Costmap2D
~~~

Costmap 配置文档列出的默认插件组合包含 static_layer、obstacle_layer 和 inflation_layer；每个插件需要在参数中声明自己的 plugin 类型。详见[Costmap 2D 官方配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)。

### 3.2 StaticLayer 与 ObstacleLayer

**官方事实：**

- StaticLayer 从 map_server 或 SLAM 接收地图并把静态障碍放入代价地图，说明见[Static Layer 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/)。
- ObstacleLayer 可以使用二维激光、深度或其它传感器的观测，通过 ray tracing 做 marking 和 clearing，说明见[Obstacle Layer 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/)。
- 因此，静态地图适合表达墙体等长期不变环境；传感器层适合把运行时出现的物体加入局部或全局代价地图。代价地图的更新频率、观测范围、未知空间策略和 footprint 配置会直接影响规划结果，相关参数在[Costmap 2D 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)中定义。

### 3.3 InflationLayer 的作用

**官方事实：** InflationLayer 在障碍物周围产生随距离衰减的代价。障碍物内以及机器人完全内切区域附近保持致命代价，远处逐渐降低；其作用不仅是把障碍“扩大”，还为规划器提供远离障碍的代价势场。官方文档见[Inflation Layer 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/)，实现细节可在[Nav2 Jazzy InflationLayer API 源码](https://api.nav2.org/nav2-jazzy/html/inflation__layer_8cpp_source.html)中核对。

关键参数包括：

- inflation_radius：障碍物代价向外传播的范围。
- cost_scaling_factor：代价随距离衰减的速度。
- robot_radius 或 polygon footprint：用于碰撞和内切区域判断；非圆形机器人不应只用一个过小的圆半径近似。

**本项目推断：** 当前 mini_nav 的“无膨胀”不仅会让路径贴墙，还使规划器没有任何“离墙远近”的信息。即使把 A* 改成 8 邻域，如果仍然只把障碍当作 0/1，路径质量也只能有限改善。应先明确 lethal threshold、未知空间、机器人几何尺寸和膨胀代价，再讨论搜索器本身。

### 3.4 静态地图与动态障碍的边界

| 问题 | 主要官方组件 | 对全局规划器的含义 |
|---|---|---|
| 已知墙体和固定障碍 | StaticLayer + 全局代价地图 | 规划器在规划时读取当前静态代价 |
| 运行中被传感器发现的障碍 | ObstacleLayer + 局部/全局代价地图 | 代价图可以更新，但是否重新规划由上层行为/调用策略决定 |
| 机器人当前能否绕开动态物体 | Controller Server + 局部代价地图 | 控制器负责短时局部可行控制，不能把全局规划器当作速度控制器 |
| 路径被动态障碍长期阻断 | Planner Server + 重新调用规划动作 | 需要重新规划或行为树策略，不能只依赖一次性全局路径 |

最后两行是基于 Nav2 官方“全局规划、局部控制、环境表示”职责的**本项目推断**，不是某个单独规划器自动提供的保证。Nav2 的[导航服务器概念文档](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/)明确区分 Planner Server 和 Controller Server；[ObstacleLayer 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/)说明了传感器观测如何进入代价地图。

## 4. Jazzy 官方规划器逐项调查

### 4.1 总览表

下表优先采用 Jazzy 官方[导航插件注册表](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)的分类；运动学适配还结合官方[首次配置导航插件指南](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/)的限制说明。

| 规划器 | 官方核心机制 | 路径平滑/方向特征 | 最小转弯半径与运动学 | 对差速车的适配 | 静态/动态障碍 |
|---|---|---|---|---|---|
| NavFn | 网格上的 Dijkstra 或 A* 波前展开，使用代价地图 | 不是 any-angle 或 SE(2) 规划器；可通过代价场影响路径形状 | 不显式搜索朝向和最小转弯半径，采用二维/圆形近似 | 适合圆形、可原地旋转的差速底盘 | 规划时读取当前代价地图；动态更新和重规划依赖外部 costmap/控制链 |
| SmacPlanner2D | 4 或 8 邻域的代价敏感二维 A*，支持降采样和简单平滑 | 有内置简单平滑器；8 邻域可减少纯直角折线，但不是运动学可行路径 | 不显式保证最小转弯半径，仍是二维网格规划 | 当前圆形差速车的首选官方对照方案 | 能利用静态、障碍和膨胀代价；动态避障仍依赖代价图、控制器和重规划 |
| SmacPlannerHybrid | 带朝向的 SE(2) Hybrid-A*，使用 Dubins/Reeds-Shepp 等运动模型 | 运动原语和专用平滑器共同生成更可执行的路径；可通过插值减少角度量化锯齿 | 显式使用 minimum_turning_radius；进行 footprint 碰撞检查 | 官方注册表列出差速支持；对圆形差速车通常只有在确实需要运动学约束时才值得引入 | 同样读取代价地图；支持动态场景的前提仍是更新 costmap 和及时重规划 |
| SmacPlannerLattice | 基于离线预生成最小控制集的 State Lattice | 路径由控制集决定，运动原语天然带方向/曲率；可配置反向扩展 | 最小转弯半径固定在预生成控制集中，不是普通运行时参数 | 适合非圆形差速或需要自定义控制集的机器人 | 可使用当前代价地图；静态地图适合启用启发式缓存，动态场景需谨慎缓存并配合更新/重规划 |
| ThetaStarPlanner | A* 的 any-angle / Lazy Theta* 变体，使用 4/8 邻域和 line-of-sight | 能跳过栅格拐角生成任意角度线段，通常比四邻域路径更直、更少锯齿 | 不保证最小转弯半径和完整运动学可行性 | 适合把圆形差速车近似为二维粒子；路径仍需控制器可跟踪 | 依赖代价地图和通视检查；动态障碍处理依赖障碍层、控制器和重规划 |

表中算法机制来自[Nav2 Jazzy 插件注册表](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)、[NavFn 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/)、[Smac 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/)和[Theta* 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/)。关于动态障碍需要 costmap/controller/replanning 协作的判断是本项目推断。

### 4.2 NavFn

**官方事实：** NavFn 是波前型全局规划器，可以使用 Dijkstra 或 A*；其 use_astar 默认关闭时使用 Dijkstra。它假设二维全向粒子，通常把机器人近似为圆形，并在加权 costmap 上搜索。重要参数还包括 tolerance、allow_unknown 和 use_final_approach_orientation。详见[Configuring NavFn Planner（Jazzy）](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/)和[NavFn Jazzy 官方源码/README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_navfn_planner)。

**路径表现：** Nav2 官方调优文档把 NavFn 描述为倾向于产生较宽的扫掠式曲线；它的路径形状受代价地图势场影响，不应理解成具有最小转弯半径的轨迹。见[Nav2 调优指南](https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/)。

**本项目建议：** NavFn 适合作为“官方最小基线”和代价地图接线验证对象，不是解决当前卡顿/锯齿路径的唯一答案。若目标是学习二维代价敏感 A*，SmacPlanner2D 与当前 mini_nav 的结构更接近。

### 4.3 SmacPlanner2D

**官方事实：** SmacPlanner2D 是代价敏感的二维 A*，支持 4 或 8 连接邻域、代价地图降采样、未知空间策略、最大迭代/规划时间和内置简单平滑器。cost_travel_multiplier 用于惩罚高代价区域：提高该值会更倾向于远离高代价区域，设为 0 时可近似二值障碍 A*。详见[Smac 2D 配置文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/)。

**运动学边界：** 它仍在二维栅格上规划，不在搜索状态中完整表达朝向、曲率和速度，因此不能保证差速车的每个中间姿态都能直接执行。官方首次配置指南把网格规划器推荐给圆形差速/全向机器人，并提醒存在转向约束的机器人需要更合适的规划器。参见[官方导航插件选择指南](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/)。

**本项目建议：** SmacPlanner2D 是当前 mini_nav 最自然的第一升级目标。它能逐项对应当前缺失项：四邻域可扩展为八邻域、均匀步长可变为代价敏感、无后处理可增加简单平滑、静态二值地图可升级为带膨胀的 Costmap2D。它仍然不替代后续的 footprint/运动学验证。

### 4.4 SmacPlannerHybrid

**官方事实：** SmacPlannerHybrid 是 SE(2) Hybrid-A* 规划器，以位置和离散朝向构成搜索状态，使用 Dubins 或 Reeds-Shepp 运动模型，并可以使用 minimum_turning_radius、完整 footprint 碰撞检查、运动原语插值和专用平滑。Jazzy 文档中的 minimum_turning_radius 默认值为 0.4 m，但项目不能直接照搬这个值，必须按底盘运动学和控制器能力配置。详见[Smac Hybrid-A* 配置文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)。

官方文档还说明，允许运动原语插值可以缓解角度离散化造成的锯齿；二次代价惩罚可提高平滑性，但可能让路径更靠近障碍物，因此必须结合膨胀和 footprint 检查调参。仍见[Smac Hybrid-A* 参数说明](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)。

**适配判断：** Hybrid-A* 对 Ackermann、腿式或其它受最小转弯半径约束的平台尤其有价值。Jazzy 官方插件注册表列出的支持类型还包括 Differential 和 Omnidirectional；不过官方入门选择指南重点把它放在非圆形/有明显运动学约束的平台。对当前圆形差速车，本项目应先证明二维规划器不够，再引入 Hybrid-A*，否则会把调试重点从代价地图和接口问题提前转移到运动原语、朝向离散和性能上。

### 4.5 SmacPlannerLattice

**官方事实：** SmacPlannerLattice 使用预先生成的最小控制集构成 State Lattice。控制集可以根据最小曲率、原地旋转、全向移动等机器人特征生成；运行时扩展的是这些控制原语，而不是简单的栅格邻居。最小转弯半径固定在预生成控制集中，不是普通运行时参数。详见[Smac Lattice 配置文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)和[Smac 官方 README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smac_planner)。

Lattice 支持反向扩展、反向代价、改变方向惩罚、非直线惩罚、代价惩罚和旋转惩罚等配置；由于控制集和地图分辨率绑定，它不像 SmacPlanner2D 那样使用普通 costmap downsampler。相关参数和限制见[Smac Lattice Jazzy 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)。

**适配判断：** Lattice 适合非圆形差速/全向机器人、需要自定义控制集的底盘，或希望把运动学可执行性编码进搜索的场景。对目前使用官方差速小车模型、主要问题是四邻域折线和代价过于粗糙的 mini_nav，它的复杂度明显高于第一阶段需要。

### 4.6 Theta*：Jazzy 仍支持

**官方事实：** ThetaStarPlanner 在 Nav2 Jazzy 中仍是官方插件。Jazzy 迁移指南列出 nav2_theta_star_planner::ThetaStarPlanner，Jazzy 官方源码目录也存在对应包，见[迁移指南](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/iron/Iron/)和[Theta* Jazzy 源码/README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_theta_star_planner)。

Theta* 通过父节点和当前节点之间的 line-of-sight 检查跳过不必要的栅格拐点，生成 any-angle 路径；how_many_corners、w_euc_cost 和 w_traversal_cost 控制邻域、路径长度和穿越代价的权衡。提高欧氏距离权重会偏向直线/紧凑路径，提高 traversal cost 权重会更重视远离高代价区域。参数详见[Theta* 配置文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/)。

**运动学边界：** 官方 README 明确说明 Theta* 不是运动学可行规划器；它仍然把机器人看作二维粒子。因此它能显著改善四邻域路径的折线外观，但不能保证最小转弯半径、连续曲率或非圆形 footprint 下的可执行性。官方 README 还建议把它和能形成可执行轨迹的局部规划器/控制器一起使用。见[Theta* Jazzy README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_theta_star_planner)。

**本项目建议：** 可以把 Theta* 作为一个清晰的研究分支：先在同一张带 InflationLayer 的地图上，与当前 A* 和 SmacPlanner2D 比较“路径长度、拐点数量、离障碍距离、控制器跟踪效果”。不要把 Theta* 的路径更直误判为差速车的运动学规划已经完成。

## 5. 当前 mini_nav 与官方方案的差距

以下结论来自当前仓库代码，而不是对官方行为的推测：

- [astar_navigator.cpp](../mini_nav_core/src/navigator/astar_navigator.cpp) 使用四个正交方向、Manhattan 启发式和每步统一的 g 代价；除致命障碍外，不同非致命代价不会影响路径选择。
- [costmap_publisher.cpp](../mini_nav_nodes/src/costmap_publisher.cpp) 把地图消息转换为当前项目的简单 Costmap2D 并触发规划；当前链路没有 Nav2 Planner Server 的 pluginlib/action 边界。
- 当前规划结果是栅格单元序列转换出的简化路径，算法层没有完整的机器人 footprint 碰撞检查、InflationLayer 代价场、路径平滑、曲率约束或动态传感器障碍更新。
- 当前主入口和官方仿真/地图属于启动与集成层；这次研究不建议把官方 Gazebo、地图或模型替换成自绘资产。官方示例入口可与当前自研入口分开维护，具体以[mini_nav_bringup/launch](../mini_nav_bringup/launch/)中的项目文件为准。

### 5.1 能力差距对照表

| 当前 mini_nav | 官方方案中的对应能力 | 影响 |
|---|---|---|
| 四邻域 | SmacPlanner2D 可选 4/8 邻域；Theta* 可做 any-angle；Hybrid/Lattice 使用运动原语 | 当前路径天然是水平/垂直台阶，转弯点多 |
| 每步统一代价 | NavFn、Smac 和 Theta* 都可以读取带梯度的 costmap 代价；Smac 2D 有 cost_travel_multiplier | 当前搜索不会主动远离膨胀区或高风险区域 |
| 只有致命障碍判断 | StaticLayer、ObstacleLayer、InflationLayer 和 footprint/collision checker | 当前路径可能擦墙，且没有安全距离概念 |
| 无 InflationLayer | 官方 InflationLayer 提供障碍周围的指数衰减代价 | 即使搜索器支持代价，输入仍然没有“离墙远近” |
| 无平滑/后处理 | Smac 规划器可带简单平滑；Nav2 还有 Smoother Server 和 SmoothPath action | 当前路径折点和方向突变直接交给后续使用者 |
| 无明确路径朝向 | nav_msgs/Path 每个点是 PoseStamped；部分官方规划器提供最终接近方向参数 | 当前栅格点序列不足以表达可靠的差速车朝向序列 |
| 无运动学状态 | Hybrid-A* 将朝向和转弯半径纳入搜索；Lattice 使用控制集 | 当前 A* 只回答“网格上能走”，不回答“底盘能否这样开” |
| 静态地图快照为主 | StaticLayer + ObstacleLayer 可组合静态与传感器观测 | 当前规划不会自然响应运行中的新障碍物 |
| 自定义话题触发一次规划 | Planner Server + ComputePathToPose + nav2_core::GlobalPlanner | 无法直接复用官方规划器选择、取消、错误状态和 Nav2 action 链 |

### 5.2 对“卡顿”和“路径不够好”的解释

**本项目推断：** 当前表现差不应只归因于 A* 的优先队列实现。至少有四个结构性原因：

1. 四邻域把连续平面离散成直角折线，机器人沿路径跟踪时需要频繁改变方向。
2. 统一代价和无膨胀让搜索器看不见“靠墙风险”，路径可能贴着障碍边缘走，控制器需要不断修正。
3. 没有平滑和连续朝向，路径点之间可能有突变；即使 RViz 中路径能显示，控制器跟踪它时也可能出现速度抖动。
4. 规划节点是面向演示的自定义一次性链路，还没有官方 Planner Server、局部代价地图、控制器和重规划机制的配合。

其中第 1 至第 3 项可以通过同一张地图、同一控制器和可记录的 nav_msgs/Path 做 A/B 测试；第 4 项是系统架构差距，不宜只在搜索循环里打补丁。

## 6. 面向 mini_nav 的升级计划

下面每一步都明确区分官方事实与本项目建议。

### 阶段 0：建立官方可重复基线

**官方事实：** Planner Server 可以配置多个全局规划器插件，并通过规划请求选择插件；官方教程和插件注册表给出了 NavFn、SmacPlanner2D、SmacPlannerHybrid、SmacPlannerLattice、ThetaStarPlanner 的配置入口。依据：[Planner Server](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/)、[导航插件注册表](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)。

**本项目建议：**

- 保留现有官方 Gazebo、机器人模型和地图作为测试基线。
- 单独写一个 mini_nav 自己的实验入口，和原有官方入口分开；只替换 AMCL/规划相关自研节点，不改地图来源。
- 先用官方 NavFn 和 SmacPlanner2D 输出 ComputePathToPose 结果，记录规划时间、路径长度、路径点数、最小障碍距离和控制器跟踪表现。
- 这一步的目标是确认差异来自规划器/代价地图，而不是地图、TF、传感器或仿真时序。

### 阶段 1：先修正代价地图语义

**官方事实：** 规划器依赖 Costmap 2D；StaticLayer、ObstacleLayer 和 InflationLayer 分工提供静态障碍、传感器障碍和安全代价，footprint/robot radius 参与碰撞判断。依据：[Costmap 2D](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)、[InflationLayer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/)。

**本项目建议：**

- 在 mini_nav 自己的 Costmap2D 语义中明确 FREE_SPACE、UNKNOWN、INSCRIBED 和 LETHAL_OBSTACLE 的边界。
- 明确 allow_unknown 策略，不要在“未知”和“空闲”之间隐式转换。
- 按官方小车的实际 footprint 或 robot_radius 做膨胀和碰撞检查。
- 为越界、未知、致命障碍、贴障碍路径和无路径补充单元测试。

没有这一步，后面的“更聪明搜索”会建立在错误输入上。

### 阶段 2：实现 SmacPlanner2D 风格的二维升级

**官方事实：** SmacPlanner2D 提供 4/8 邻域、代价敏感搜索、代价旅行倍率、降采样和简单平滑等能力；它仍不保证完整运动学可行性。依据：[Smac 2D 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/)。

**本项目建议：**

- 保持当前 A* 的小而清晰的接口，先把邻域策略从固定四邻域扩展为可配置 4/8 邻域。
- 对对角移动使用几何距离代价，并禁止穿过障碍物拐角。
- 将每个候选单元的 travel cost 纳入 g 值；高代价区域不应与自由空间等价。
- 规划结果生成 PoseStamped，至少根据相邻路径点切线计算连续的 yaw，并单独处理终点朝向。
- 搜索和路径后处理分层，方便分别测量搜索耗时、平滑耗时和碰撞检查耗时。

### 阶段 3：加入可验证的平滑器

**官方事实：** Nav2 提供 Smoother Server 和 SmoothPath action；官方插件注册表列出 Simple Smoother、Constrained Smoother 和 Savitzky-Golay 等平滑器。简单平滑器适合改善二维规划器的折线，约束平滑器才适合把平滑、障碍距离和最小转弯半径一起作为约束问题处理。依据：[导航插件注册表](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)和[SmoothPath API](https://api.nav2.org/actions/jazzy/smoothpath.html)。

**本项目建议：**

- 第一版只做简单、可解释的 shortcut/梯度类平滑。
- 每次修改路径后重新采样并进行 footprint 碰撞检查；平滑失败时回退到原始安全路径。
- 把“看起来更直”和“机器人可以安全跟踪”作为两个独立指标，不能只看 RViz 画面。

### 阶段 4：接入官方 Planner Server 插件边界

**官方事实：** 自定义规划器通过 nav2_core::GlobalPlanner、pluginlib XML 和 Planner Server 参数映射加载。依据：[官方插件教程](https://docs.nav2.org/jazzy/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/)。

**本项目建议：**

- 保留 ROS 无关的 AStarPlanner/后续 Smac2D 风格算法核心。
- 新增一个很薄的 ROS 插件适配层，负责参数、TF、costmap wrapper、生命周期和 nav_msgs::msg::Path。
- 让算法核心不直接发布 /cmd_vel，也不直接管理 action；路径跟踪交给官方 Controller Server 或专门的控制节点。
- 用 ComputePathToPose 做接口验收：有效起点/终点、越界、占用目标、无路径、取消和超时都要有明确结果。

### 阶段 5：接入动态障碍与重规划

**官方事实：** ObstacleLayer 从传感器观测更新代价地图，Controller Server 使用局部代价地图执行控制；Planner Server 负责全局规划。依据：[ObstacleLayer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/)、[导航服务器概念](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/)。

**本项目建议：**

- 先让局部代价地图和控制器正确避开短时动态障碍，再决定是否提高全局重规划频率。
- 对静态或变化很慢的 costmap 才考虑使用 Smac Hybrid/Lattice 的障碍启发式缓存；动态环境中要验证缓存失效和代价图更新是否匹配。官方对缓存加速的说明见[Smac Hybrid 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)和[Smac Lattice 文档](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)。
- 以 rosbag 或固定场景记录动态障碍出现、代价图更新、重规划请求、控制器减速和恢复行为的时间顺序。

### 阶段 6：按真实运动学选择 Hybrid 或 Lattice

**官方事实：** Hybrid-A* 显式搜索带朝向的运动状态和最小转弯半径；Lattice 使用离线最小控制集，最小转弯半径由控制集固定。依据：[Smac Hybrid](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)和[Smac Lattice](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)。

**本项目建议：**

- 若机器人仍是可原地旋转、近似圆形的差速小车，停留在 SmacPlanner2D + 合理控制器即可，不要为了路径视觉平滑过早增加 Hybrid/Lattice。
- 若机器人变为非圆形差速底盘，需要严格 footprint 和受限转向，优先研究 Lattice 控制集。
- 若机器人需要明确的最小转弯半径、前进/倒车运动模型或连续朝向搜索，再研究 Hybrid-A*。
- 选择标准应是可执行性测试和安全距离，不是算法名称或路径在 RViz 中是否“好看”。

## 7. 适用版本、查询说明与限制

- 本文以 **ROS 2 Jazzy / Nav2 Jazzy** 为适用版本，链接优先固定到 docs.nav2.org/jazzy、api.nav2.org/.../jazzy 和 Navigation2 GitHub 的 jazzy 分支。
- Theta* 的 Jazzy 支持通过 Jazzy 官方文档、Jazzy 迁移指南和 nav2_theta_star_planner Jazzy 源码目录交叉确认；不要用 Rolling/main 分支的新增接口反推 Jazzy 行为。
- nav2_core::GlobalPlanner 的接口描述采用 ROS 2 Jazzy 官方 API 索引和 Jazzy 官方插件教程；后续如果切换发行版，应重新核对纯虚函数签名。
- 官方插件注册表和首次配置指南的适配描述粒度并不完全相同：注册表给出各插件列出的支持机器人类型，入门指南还给出“二维网格规划器不适合有明显转向约束平台”的工程警告。本文保留这个区别，没有把“列出支持”解释为“任何底盘都无需验证”。
- 本文没有把官方 README 中的示例性能数字当作 mini_nav 的性能承诺。mini_nav 的规划时间、路径质量和跟踪效果必须在同一 Gazebo、地图、机器人模型和控制器配置下实测。
- 本文未修改源代码、启动文件、地图、Gazebo 资产或其它文档，也未执行构建和 Git 提交。

## 8. 官方来源清单

### Nav2 Jazzy 架构与插件

- [Planner Server 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_planner_server/)
- [Navigation Servers 概念](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/navigation_servers/)
- [导航插件注册表](https://docs.nav2.org/jazzy/configuration_and_development/navigation_plugins/)
- [首次配置导航插件指南](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/navigation_plugins/setup_navigation_plugins/)
- [编写全局规划器插件教程](https://docs.nav2.org/jazzy/tutorials/plugin_tutorials/writing_new_planner_plugin/writing_new_planner_plugin/)
- [nav2_core::GlobalPlanner ROS 2 Jazzy API](https://docs.ros.org/en/jazzy/p/nav2_core/generated/file_include_nav2_core_global_planner.hpp.html)
- [Navigation2 nav2_planner Jazzy 源码目录](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_planner)
- [Jazzy 迁移指南中的 Planner Server 插件](https://docs.nav2.org/jazzy/configuration_and_development/migration_guides/iron/Iron/)

### 官方规划器

- [NavFn 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/configuring_navfn/)
- [NavFn Jazzy 源码/README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_navfn_planner)
- [Smac 规划器总览](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/)
- [SmacPlanner2D 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_2d/configuring_smac_2d/)
- [SmacPlannerHybrid 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_hybrid/configuring_smac_hybrid/)
- [SmacPlannerLattice 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/smac/smac_lattice/configuring_smac_lattice/)
- [Smac 规划器 Jazzy 源码/README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_smac_planner)
- [ThetaStarPlanner 配置](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/)
- [Theta* Jazzy 源码/README](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_theta_star_planner)
- [Nav2 Jazzy 调优指南](https://docs.nav2.org/jazzy/configuration_and_development/tuning_guide/)

### 代价地图与障碍物

- [Costmap 2D 配置指南](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/)
- [环境表示](https://docs.nav2.org/jazzy/getting_started/navigation_concepts/environmental_representation/)
- [Static Layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/)
- [Obstacle Layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/obstacle/)
- [Inflation Layer](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/)
- [InflationLayer Jazzy API 源码](https://api.nav2.org/nav2-jazzy/html/inflation__layer_8cpp_source.html)

### ROS 2 / Nav2 路径接口

- [ROS 2 Jazzy nav_msgs README](https://docs.ros.org/en/jazzy/p/nav_msgs/README.html)
- [Nav2 Jazzy action API 索引](https://api.nav2.org/jazzy/actions/)
- [ComputePathToPose action](https://api.nav2.org/actions/jazzy/computepathtopose.html)
- [FollowPath action](https://api.nav2.org/actions/jazzy/followpath.html)
- [SmoothPath action](https://api.nav2.org/actions/jazzy/smoothpath.html)

