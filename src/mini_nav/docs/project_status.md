# mini_nav 项目状态

最后更新：2026-10-06
目标平台：ROS 2 Jazzy

`mini_nav` 是用于逐步理解移动机器人导航链路的自研学习项目。它以小而可验证的模块推进，并在后期与 `nav2_learning` 中的官方 Nav2 案例对照；`nav2_learning` 不属于本项目的运行依赖。

本次依据 `logs/` 下全部 15 个日期目录（2026-08-27 至 2026-10-06）补充。下文区分调研方案、已实现能力、实际验收及撤回实验；历史记录中的参数、测试数量和未完成项只表示对应轮次。测试与仿真结果来自已有日志，本次文档整理未重新构建、测试或运行机器人。

## 当前状态摘要（截至 2026-10-06）

- 已形成自研基础单目标导航闭环：定位、全局规划、路径跟踪、局部障碍处理、重规划、取消/抢占、有限失败与独立失联停车。不能再把当前项目描述为只有静态路径可视化。
- 定位有三个独立使用方向：默认自研 AMCL；FAST-LIVO2 融合前端加 GICP 旧图定位；SCURM FAST-LIO2 的 ICP 初始化和固定先验定位。后两者复用自研导航核心，均保留人工初值和质量/会话失效门控。
- 显示膨胀已对齐 Nav2 learning；安全判定采用独立的连续车体几何。最新修复保留静态地图定位预算，同时将实时 LaserScan 端点留在 `odom` 检查相对距离，避免重复叠加绝对地图定位误差。
- 最新日志验收为 **103 个实际用例通过**（core 75、nodes 27、RViz 1）；隔离 Gazebo 中 7 次任务成功、7 次停车后重新规划成功。该结果对应 10 月 6 日的有限场景，不代表所有定位后端、地图和长期运行均已验收。见 [复发问题评估](../logs/26-10-6/recurrent_failure_assessment.md)及[测试计数](../logs/26-10-6/recurrent_failure/current_test_counts.json)。

| 使用方向 | 入口 | 当前边界 |
|---|---|---|
| 默认 AMCL 导航 | `ros2 launch mini_nav_bringup mini_localization_astar.launch.py` | 自研 AMCL 与导航核心；默认域 61、分区 `mini_nav` |
| FAST-LIVO2 建图 / 旧图导航 | `fastlivo_mapping.launch.py` / `fastlivo_navigation.launch.py`（`mini_nav_bringup`） | 建图与导航独立启动；绑定三维/二维资产，导航只读旧图、人工初值 GICP；见 [包说明](../mini_nav_fastlivo/README.md) |
| SCURM FAST-LIO2 先验导航 | `ros2 launch scurm_sim fastlio2_navigation.launch.py` | 使用 `install_mini_nav_fastlio`；默认域 227、分区 `scurm_mini_nav`；见 [定位后端说明](localization_backends.md) |

构建完成不会热替换已有进程。10 月 6 日增加了 `CollisionMap.points_frame_id`，使用该修复需一起重启整套导航 launch，再初始化定位；日志中保留的现场进程不能视为已自动升级。

## 文档入口

- [架构说明](architecture.md)：模块职责、依赖方向、ROS 接口、TF 和启动链路。
- [贡献指南](contributing.md)：修改位置、代码风格、测试命令、运行验收和提交检查清单。
- [官方 AMCL 代码导读](../logs/26-8-30/amcl_code_walkthrough.md)：从节点启动、地图和初始位姿，到粒子更新、激光模型和 `map -> odom` 发布的源码承接关系。
- [规划代价地图](planning_costmap.md)：障碍膨胀、安全余量、A* 代价与 RViz 显示。
- [单目标导航任务](navigation_tasks.md)：Action、重规划、定位门控、取消和速度保护。
- [局部代价地图](local_costmap.md)：滚动窗口、观测清除、连续端点与失效策略。
- [定位后端目录说明](localization_backends.md)：AMCL / FAST-LIO2 源码归属及独立构建入口。

## 当前单目标任务版本（2026-09-30）

主入口已接入 `NavigateToPose -> ComputePathToPose -> FollowPath` Action 任务链、取消/抢占、原始激光融合重规划、有限失败期限、AMCL 主簇与定位质量门控、独立命令看门狗、统一任务暂停入口和 RViz 反馈/取消按钮。

使用说明与边界见 [单目标导航任务](navigation_tasks.md)，本轮验证记录见 [实施报告](../logs/26-9-30/navigation_completion_implementation.md)。下方按日期保留以前的完成记录；旧记录中的测试数量和未完成项表示当时状态。

## FAST-LIVO2 双入口原型（2026-10-03）

新增独立 `mini_nav_fastlivo` 包，建图入口保存三维定位图与二维高度导航图；导航入口独立重启后只读加载旧图，经人工初值和 GICP 先验约束定位，复用原导航 Action 与核心。现有平地 Gazebo 的原起点、偏移起点导航及前端失效停车通过；尚未验收实车、全局自动重定位和长程精度。

启动步骤见 [使用说明](../mini_nav_fastlivo/README.md)，改动、验收证据和 RViz 截图限制见 [实施报告](../logs/26-10-3/fastlivo_two_launch_implementation.md)。AMCL 原入口保留。

## 当前已完成

SCURM FAST-LIO2 先验定位已接入自研导航并支持 `/initialpose`。定位源码按 `mini_nav_core/src/localization/amcl` 和 `fastlio2` 整理，ROS 接入和独立构建位置见 [定位后端目录说明](localization_backends.md)。该入口使用 FAST-LIO2，不启动 LIVO2；默认 AMCL 入口保留。

- `mini_nav_core`
  - 二维静态代价地图 `Costmap2D`。
  - 栅格坐标与世界坐标之间的转换。
  - 八邻域 A*、连续车体安全校验、平滑路径和容差内最近可达终点。
  - 静态障碍物膨胀、安全区与软代价 A* 规划。
  - 代价地图与 A* 规划器的单元测试。
  - `localization/` 中的 `LocalizationMap`、差分运动模型、激光模型、`PoseBinIndex` 和 `ParticleFilter`。
  - 规划、后处理与跟踪器共用连续圆盘扫掠；静态格按面积检查，实时二维扫描保留连续端点。
  - 安全起点可连接所属或八邻接格中心；终点检查覆盖控制器允许的 0.12 m 停车圆盘。
  - 小规模差分候选运动选择、反应/制动预测与有界避障；没有安全候选时停车。
- `mini_nav_nodes`
  - 从 map_server 的 `/map` 接收 `nav_msgs/msg/OccupancyGrid`，转换为 `Costmap2D`。
  - 通过 `map_file` 参数直接加载标准 trinary YAML/PGM 地图，并发布 `/mini_nav/map`。
  - 收到 RViz `/goal_pose` 时，从机器人当前 `map -> base_footprint` TF 获取规划起点；`/initialpose` 使旧路径失效。
  - 将 A* 结果发布为 `/mini_nav/global_path`。
  - 独立发布 `/mini_nav/planning_costmap`，不改变原始地图和 AMCL 输入。
  - `AmclNode` 生命周期节点及 `/amcl_pose`、`/particle_cloud`、`map -> odom` 适配。
  - AMCL 定位更新与 TF 发布解耦：未达运动阈值时，仍按激光时间戳重发缓存的 `map -> odom`。
  - 单目标任务管理、三个标准 Action、任务暂停、取消/抢占、动态受阻重规划与有限超时。
  - `CollisionMap` 原子传递原始格、连续端点、观测帧、误差预算和有效性；失效时撤销运动许可。
  - 独立 `velocity_guard_node` 唯一发布 `/cmd_vel`，命令/任务心跳超时、时钟异常和非法速度归零。
- 三维定位与地图实验
  - `mini_nav_fastlivo` 建图/旧图导航、成对资产校验、完整会话记录和离线概率射线重放。
  - SCURM FAST-LIO2 固定先验定位、`/initialpose` 与后端会话重启；源码和独立部署归入 `mini_nav`。
- 可视化
  - 已有 RViz 配置可显示静态地图、全局路径和地图坐标轴。
  - 导航状态面板提供任务反馈和取消；PS5 面板提供手动开关和限速。最新避障状态显示为“正在避障”。

## 历史完成记录

以下保留原有阶段记录；其中“尚未接入”“未验收”等描述不用于判断后续版本。全日期日志补充及当前边界见后文。

### 默认地图切换（2026-09-30）

- 默认地图改为 `scripts/run_nav2_case.sh` 第一个 `learning` 案例使用的 `tb3_learning.yaml/pgm`，原样复制到 `mini_nav_bringup/maps/`；地图为 `112 × 103`、`0.05 m/格`，原点为 `(-0.961, -2.072)`。
- 自研定位入口、官方定位对照入口与独立 A* 脚本统一使用新地图；两套 launch 继续支持 `map:=...` 覆盖。
- `mini_nav_bringup` 构建、两套 launch 的 `--show-args`、Python/Bash 语法及源码/安装地图一致性检查通过；该资源包的 `colcon test` 未发现注册测试（0 项）。本轮未启动 Gazebo 或运动节点，新地图上的定位和导航效果尚未验收。

### 完成日期：2026-08-22

- 完成 `Costmap2D` 的输入校验、原子式重建和越界保护；补充全图填充、直线、矩形、边界及索引/坐标转换便利接口。
- 直线绘制支持四邻域连续的斜线障碍，避免 A* 从仅角点相接的障碍格之间穿过。
- 旧的迷宫绘制逻辑已移除；当前地图由官方示例地图和 Nav2 `map_server` 提供，`main.cpp` 不绘制地图。
- 地图坐标轴 Marker 按实际世界尺寸与地图原点缩放；启动脚本从首张 `OccupancyGrid` 自动计算 RViz 的初始距离和焦点，并拒绝已有地图发布者以避免显示跳动。
- 已验证 `mini_nav_core`、`mini_nav_nodes` 构建成功；`Costmap2D` 9 项单元测试与 A* 1 项单元测试通过。

### 完成日期：2026-08-23

- 在 `mini_nav_bringup` 增加 `waffle_sim.launch.py`，复用系统安装的 `turtlebot3_gazebo`、`ros_gz_sim` 和 `ros_gz_bridge` 启动官方 TurtleBot3 Waffle。
- 启动文件固定使用 Waffle，并支持 `use_sim_time`、`x_pose` 和 `y_pose` 参数；`--show-args` 已确认加载 `turtlebot3_waffle.urdf`。
- 已验证 `colcon build --packages-select mini_nav_bringup` 构建成功。
- 已验证 `/clock`、`/scan`、`/odom`、`/tf`、`/joint_states` 的消息类型和实际数据；`odom -> base_footprint` TF 正常。
- 已验证 `/cmd_vel` 类型为 `geometry_msgs/msg/TwistStamped`，`ros_gz_bridge` 已建立 ROS 到 Gazebo 的订阅接口；本项目没有非零速度发布者。
- 已确认没有 `map -> odom` 变换，避免把静态 TF 当作定位结果。
- Gazebo 内置 teleop 直接使用 Gazebo Transport，机器人运动时不保证在 ROS 2 `/cmd_vel` 中产生回显；这不影响 ROS 侧速度接口的桥接验收。

- `mini_nav_nodes` 已移除硬编码迷宫，按 `/map` 的尺寸、分辨率和原点重建内部代价地图；占用栅格障碍和未知区域均禁止 A* 通过。
- 已验证 `mini_nav_core`、`mini_nav_nodes` 构建成功；`mini_nav_core` 的 10 项测试及 `mini_nav_nodes` 包测试通过，A* 启动文件 `--show-args` 正常。
- 已验证 `costmap_publisher_node` 直接加载 `maps/turtlebot3_map.yaml`，得到 `107 × 107`、`0.05 m/格`、原点 `(0.724, -3.997)` 的地图并进入发布状态。
- `start_astar_navigation.sh` 已传入该 `map_file`，A* 演示不再依赖单独的 `/map` 发布者。

### 完成日期：2026-08-27

- 在 `mini_nav_bringup` 增加 `official_localization_astar.launch.py` 作为官方 AMCL 对照入口，并增加 `mini_localization_astar.launch.py` 作为自研 AMCL 入口；两者复用官方 Waffle、Gazebo、map_server、地图和 RViz 资源。
- 增加项目自有 `amcl_waffle.yaml`，只包含 AMCL 参数，不再使用会覆盖地图路径的完整 TurtleBot3 导航参数文件。
- 将当前 `turtlebot3_map.yaml/pgm` 安装为 bringup 包的默认地图资源；已验证 map_server 加载 `107 × 107`、`0.05 m/格` 地图并进入 active 状态，AMCL 也成功收到地图并进入 active 状态。
- `sim_check.launch.py` 不再发布静态 `map -> odom`；该变换由 AMCL 在设置初始位姿后提供。
- 已验证两个入口的 `--show-args`、Python/YAML 静态检查和三包构建；`mini_nav_core` 的 19 项测试通过。

### 自研 AMCL ROS 接入（2026-08-30）

- `AmclNode` 已接入 `mini_nav_nodes`，生命周期管理集中在 `amcl_node.cpp`，`main.cpp` 只保留 ROS 初始化、节点构造和 spin。
- 自研入口复用官方仿真和地图，只替换 AMCL；官方入口单独保留用于对照。
- 修复定位回调组未加入 executor 的问题：`MutuallyExclusiveCallbackGroup` 已设置为自动加入 executor，地图、初始位姿和激光回调可以被普通 `rclcpp::spin()` 调度。
- 已在修改前创建 `mini_nav_nodes/src/amcl_node.cpp.orig` 原样备份；修改后重新构建三包，生命周期 configure/activate 验证通过，`colcon test-result --verbose` 为 26 项测试、0 错误、0 失败、0 跳过。
- 重启 `mini_localization_astar.launch.py` 后，已确认自研 AMCL 能接收初始位姿并发布 `/amcl_pose`、`/particle_cloud` 和动态 `map -> odom`。
- 基础定位链路已验证；Gazebo 中长时间运动、定位收敛指标和官方对照尚未完成。

### AMCL TF 平滑发布（2026-08-31）

- 对照 Jazzy 官方 AMCL 后确认，原实现在位移未达 `update_min_d` 且转角未达 `update_min_a` 时直接返回，导致 `map -> odom` 也停止刷新。旧 TF 超出 `transform_tolerance` 后，RViz 会表现为机器人暂停后跳动。
- `AmclNode` 现在保存最近一次有效 `map -> odom` 的几何值。粒子滤波更新时重新计算缓存；未达运动阈值时保持几何值不变，使用当前激光时间戳加 `transform_tolerance` 重新发布。
- 成功替换地图、设置新初始位姿、全局定位、节点 cleanup 或 shutdown 时会使旧 TF 缓存失效，避免跨定位周期复用旧变换。
- 新增 `test_amcl_tf_cache.cpp` 的 3 项测试，覆盖首次估计前禁止发布、缓存重时间戳和缓存失效。已重新构建 `mini_nav_core`、`mini_nav_nodes`、`mini_nav_bringup`；`colcon test-result --verbose` 为 30 项测试、0 错误、0 失败、0 跳过。
- 已通过 Gazebo teleop 移动 Waffle 并在 RViz 观察验收；自研 AMCL 在低速、未频繁触发粒子滤波更新时的显示已保持连续，本次卡顿问题通过。

### 静态规划代价地图（2026-09-28）

- 在当前代码提交基线 `ff6e4d2` 后，参考已下载的 Jazzy Nav2 `InflationLayer` 和 Smac 2D 代价公式，实现 ROS 无关的障碍膨胀与软代价 A*。
- 按 Waffle 碰撞盒最远角点约 0.238 m，默认取 0.24 m 机器人半径加 0.02 m 安全余量形成禁行区；确定障碍至 0.45 m 处形成渐变代价，未知格周围的硬安全带由 `inflate_around_unknown` 控制；原始 `/map` 不变，发布独立 `/mini_nav/planning_costmap` 供 RViz 查看。
- 两套定位加 A* 的 launch 加载 `planning_costmap.yaml`；三包构建和两套 launch 的 `--show-args` 已通过。
- 本次未运行单元测试或 Gazebo 运动验收；车体余量与实际通道宽度仍需结合仿真调整。

### 从机器人当前位置安全规划（2026-09-29）

- 新目标到来时从 `map -> base_footprint` TF 取当前起点，在膨胀规划图上执行 A*；路径中间点沿路径方向，末点保留目标朝向。
- 初始位姿重设后清除旧路径，等待新的 `map -> odom`；地图更新、TF 过期或无效、起终点位于硬安全区及无路时清除旧路径。
- `planning.max_pose_age` 默认 1.0 s；`base_frame_id`、`odom_frame_id` 可按 TF 树配置。
- 无机器人 TF 的独立 A* 演示显式启用手选起点参数；两套定位入口保持默认 TF 起点。
- 节点及 bringup 包已构建通过；尚未做 Gazebo 移动车辆后的路径、安全间距验收，也未运行本次改动的测试。

### 生命周期链路与滚动局部图验收（2026-09-29）

- 自研 AMCL 在 activate 时建立与 Nav2 lifecycle_manager 的 bond，在 deactivate、cleanup、shutdown 和 ROS context 预关闭时释放。隔离域中的管理器日志确认 `Server amcl connected with bond`、`Managed nodes are active`；Ctrl+C 后 AMCL 与管理器均正常退出。
- 修复滚动图整格边界的浮点截断误差，保留“累计位移满一格才滚动”的规则；新增正反方向边界回归测试。运行 `colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup` 和对应三包 `colcon test`，汇总为 86 项测试、0 错误、0 失败。
- 在隔离的 Gazebo 无界面服务端启动官方 Waffle、ROS 桥、map_server、自研 AMCL、规划图与局部图节点。临时障碍加入时，前向扫描约 0.96 m，局部图 `(0.9, 0)` 的占据值由 51 升至 100；移走后扫描恢复无有限返回，该格回到 51。
- 机器人前进约 0.294 m 后，4 m 局部窗口的 x 原点由 -2.00 m 移至 -1.75 m，仍为有效图；以上过程 `/map` 的 CRC32 始终为 `3708318207`。仅切断 `/scan` 而保持 `/clock`、`/odom` 时，`/mini_nav/local_costmap_valid` 变为 `false`，80 × 80 个栅格全部恢复为未知。
- 验收使用无界面 Gazebo；图形版 Gazebo/RViz 的窗口显示未在本轮验收。当前局部图已能感知和清除动态障碍，但尚未接入路径执行与安全停车。

### M1：自研定位核心（2026-08-27）

- 在 `mini_nav_core` 增加 ROS 无关的 `localization/` 子模块。
- 完成 `Pose2D`、`Covariance3`、`Particle`、`PoseEstimate` 数据结构。
- 完成粒子初始化、完整协方差 Cholesky 采样、权重校验/归一化、系统重采样、圆周角度均值和协方差估计。
- 新增 9 项粒子滤波单元测试；与原有地图/A* 测试一起通过，测试结果为 26 项、0 错误、0 失败、0 跳过。
- 该记录描述 2026-08-27 的核心阶段；当前已继续完成差分运动模型、激光模型、`PoseBinIndex`、ROS 适配节点和自研 launch 接入。

## 全日期日志补充

按日志日期排序；同一天先调研、再实现或撤回的记录分别说明。`before/` 和 `source_before_rollback/` 是历史源码/文档备份，不重复算作功能完成记录。日志中的运行 PID、窗口和“当前正在运行”只表示采集当时状态。

### 2026-08-27：AMCL 学习与自研定位路线

完成 [AMCL 学习资料](../logs/26-8-27/amcl_research.md)，梳理地图、激光、里程计、初始位姿、生命周期及动态 `map -> odom`，形成运动模型、传感器模型、粒子滤波与 ROS 接入的阶段路线。它是学习和设计依据，不能把官方 KLD、恢复或服务接口都计为当日自研能力；自研 M1 的实现和测试见前面的历史记录。

### 2026-08-29：AMCL 源码细节与模块分工

补充 [源码细节串联](../logs/26-8-29/technicial-detail-elaboration.md)：扫描时间 TF / MessageFilter、地图距离场、KLD 位姿分箱、协方差、主簇选择及参数归属。明确核心算法与 ROS 消息/TF 适配分离；本记录为解释资料，没有新增运行验收。

### 2026-08-30：官方 AMCL 输入到输出导读

完成 [官方代码导读](../logs/26-8-30/amcl_code_walkthrough.md)，串联生命周期、地图转换、粒子初始化、运动/观测更新、重采样、聚类和 TF 输出；引用项目内 Jazzy 参考源码，区分真实代码片段与伪代码。自研 ROS 接入、回调组修复和基础定位验证见前面的同日记录，导读本身不证明与官方全部功能等价。

### 2026-08-31：官方路径规划方案调研

[规划方案调研](../logs/26-8-31/official_path_planning_research.md)明确以 SmacPlanner2D 作为二维基线，先补安全代价语义、八邻域与后处理，再按需求评估 Theta*、Hybrid-A* 或 Lattice。动态避障还需观测层、控制与重规划；调研未修改代码或执行构建。当天 AMCL TF 连续刷新修复另见历史完成记录。

### 2026-09-02：完整导航路线与验收基线

[路径规划路线报告](../logs/26-9-2/nav2_path_planning_roadmap.md)提出官方基线、代价地图、A* 升级、路径后处理、插件边界、控制/恢复及 A/B 指标。这是当时的路线设计，后来实际采用自研 Action 状态机与导航核心；不能把报告提出的 Planner Server 插件、官方 Controller 或行为树视为已经接入。

### 2026-09-12：定位命名、数值保护与位姿推导

- [命名与魔法数字整改](../logs/26-9-12/localization_naming_and_magic_number_audit.md)修复类型定义顺序和 BeamModel 命名空间，增加 `ComposePose2D()`、`PoseBinIndex`、地图三态接口、具名常量与非有限输入保护；兼容旧接口和 ROS 参数名。
- 六个定位源文件语法检查、三包构建、launch 参数及所选包 26 个测试通过；当时全工作区汇总为 35 条记录，不能与实际用例重复累计。该轮仍未实现的主簇/随机恢复项，应按后续源码和 9 月 30 日报告判断。
- [位姿复合矩阵推导](../logs/26-9-12/pose_transform_matrix_derivation.md)解释 `T_parent_child = T_parent_base × T_base_child` 及平移/角度复合，属于学习文档，没有独立运动验收。

### 2026-09-28：从当前位置安全规划的下一步研究

[完整导航差距调研](../logs/26-9-28/navigation_completion_research.md)指出 `/initialpose` 缓存不能充当移动车辆的实时起点，提出规划时查询当前 TF、路径朝向、失效清理与安全间距验收。该报告后来标注 9 月 29 日已实现实时起点；它记录的四邻域、缺少控制器等描述属于当时快照。

### 2026-09-29：A*、后处理、局部图及闭环分工

- [A* 改进研究](../logs/26-9-29/astar_upgrade_research.md)提出八邻域、禁止穿角、连续车体校验、实际起点连接和可回退后处理；调研时未修改规划器或运行测试。
- [平滑与局部图研究](../logs/26-9-29/path_smoothing_local_costmap_research.md)区分全局几何路径和 `odom` 滚动观测图，要求射线清除、观测过期和后处理复核。[局部图后的导航缺口](../logs/26-9-29/navigation_gap_after_local_costmap_research.md)继续定义跟踪、停车、动态重规划和任务结束的职责。
- 实时起点、AMCL bond、滚动原点边界及 Gazebo 障碍出现/清除的实现验收已记录在前文；报告中的建议不能单独视为这些实现的证据。
- `map_diff_research.md` 和 `plan_next.md` 当前为空文件，没有可补充的结论或验收结果。

### 2026-09-30：基础单目标导航闭环完成

- [差距研究](../logs/26-9-30/navigation_gap_research.md)及[完成度审计](../logs/26-9-30/navigation_completion_audit.md)确认已有跟踪器，同时指出任务身份、取消、有限受阻处理、定位门控和独立停车边界的缺口。
- [碰撞误停诊断](../logs/26-9-30/collision_risk_diagnosis.md)与[修复报告](../logs/26-9-30/collision_risk_fix_report.md)将前视直线检查改为实际候选弧线扫掠；冻结快照误停由 84/84 变为 0/84，并有隔离节点闭环和单轮 Gazebo 到达停车。该轮测试为 51 个 gtest，通过不代表长期路线已验收。
- [闭环实施报告](../logs/26-9-30/navigation_completion_implementation.md)完成三个 Action、取消/抢占、激光融合重规划、20 s 受阻/无进展和 180 s 总期限、定位质量/会话门控、独立 0.35 s 命令看门狗与统一任务启停。修复局部栅格二次投影导致掉头起点被拒绝，改为原始端点单次投影。
- 最终四包构建、**60 个 gtest + 9 个隔离 ROS 集成场景，共 69 项通过**；干净隔离 Gazebo 中连续前进、前进、掉头三个任务成功并零速。到达误差按实时地图 TF 计算，不是 Gazebo 真值定位精度；重复仿真服务器污染的一轮没有计入通过结果。
- [RViz 导航面板](../logs/26-9-30/rviz_navigation_status_panel.md)与[入口环境内置](../logs/26-9-30/launch_environment_update.md)提供状态/任务控制及默认域、分区、DDS 配置。系统 RViz 完整窗口退出异常仍有记录，隔离插件测试通过不能代替整窗退出验收。速度、目标容差及入口细节保留在后面的同日历史记录。

### 2026-10-01：FAST-LIVO2 后端和独立分支研究

[接入调研](../logs/26-10-1/fastlivo_localizer_integration_research.md)、[分支方案](../logs/26-10-1/fastlivo_navigation_branch_plan.md)及[参考项目清单](../logs/26-10-1/fastlivo_reference_projects.md)固定算法/移植版本，明确真正三维 LiDAR、IMU、相机输入，以及先验地图配准、私有坐标、外参、质量和代际协议。推荐官方 FAST-LIVO2 作算法基线、RDR ROS 2 移植加 Jazzy 补丁；普通里程计或 PCD 保存不等于旧图定位。这一天仅研究，未创建分支、安装、构建或运行前端。

### 2026-10-02：FAST-LIVO2 仿真前端部署

[部署验收](../logs/26-10-2/fastlivo2_gazebo_deployment.md)在 `src/fastlivo2_deploy` 固定 RDR 提交 `837b7bbc1431cb04cf936528e52c83c835efba8e` 及依赖，完成 Jazzy 构建、Waffle 三维传感器、最终融合状态/测量时间和 QoS 适配。视觉匹配与 EKF 更新实际执行，不是 LIO-only 冒充 LIVO。

五包构建、3 个实际 gtest 通过；短程直行/转向的 249 对有效真值样本，相对位置 RMSE 0.01918 m、最大 0.02985 m，终点相对姿态误差 0.675°。这是首帧对齐后的短程仿真结果，当天尚无旧图定位或 mini_nav 导航闭环。

### 2026-10-03：累计地图、手柄、双入口与墙厚实验

- **几何和彩色地图保留。** [三维地图存盘](../logs/26-10-3/fastlivo2_persistent_mapping.md)保留完整世界扫描，体素累计、持久 QoS、原子 PCD 与独立会话目录；新增 6 个 pytest 加原 3 个 gtest 通过。修复 x86 Eigen/PCL 对齐导致的退出崩溃后，427 帧处理及退出存盘通过。[彩色地图](../logs/26-10-3/fastlivo2_persistent_color_mapping.md)保留首次 XYZ/RGB，9 个 pytest 加 3 个 gtest 通过，真实输入中历史位置/颜色、存盘和退出验证通过。累计 PCD 不恢复完整视觉/滤波器状态。
- **合并启动和手动控制。** [Gazebo/RViz 合并入口](../logs/26-10-3/gazebo_rviz_combined_launch.md)完成窗口启动及地图/轨迹数据链，但严格全部进程正常退出验收失败，系统 RViz 问题未修复。[PS5 USB 控制](../logs/26-10-3/ps5_dualforge_gazebo_control.md)复用用户驱动，L1 切换、蓝/红灯与振动获用户实测确认，唯一 guard 输出速度；2 m/s、2 rad/s 是手动指令上限，不是实际高速性能指标。[PS5 面板](../logs/26-10-3/ps5_rviz_control_panel.md)完成开关、原子限速、单轴零速、断连停车和真实 RViz 插件测试；模拟 USB 测试不能替代真实面板操作体验验收。
- **独立建图与旧图导航。** [三维到二维方案](../logs/26-10-3/fastlivo_3d_to_2d_navigation_plan.md)随后落实为[双入口原型](../logs/26-10-3/fastlivo_two_launch_implementation.md)：保存三维/二维绑定包，导航重启后只读 GICP 定位，保持一套导航核心。11 个新包用例、19 个节点用例、9 个仿真存储用例通过；短程建图、原/偏移出生点导航、前端暂停撤销任务并停车通过，地图哈希不变。尚无全局自动地点搜索或长程精度验收。
- **墙厚复现与撤回。** [两次复现](../logs/26-10-3/wall_thickness_reproduction/report.md)记录原始黑墙占用带约 11.83–11.84 cm 增至 16.16 cm，支持投影偏差、永久命中累积和二维噪声命中叠加。[在线修复实验](../logs/26-10-3/wall_thickness_fix_report.md)曾实现 GICP、分层证据、有限首帧平面融合与历史重建，18 个 pytest 和短程回归通过，但墙带增长仅勉强低于半格门限；**随后按用户要求撤回**，见[撤回说明](../logs/26-10-3/wall_fix_rollback_20261003_233723/README.md)。恢复原布尔建图和 schema 1，实验地图/记录保留，不能将撤回功能列为当前能力。
- [墙厚与膨胀研究](../logs/26-10-3/wall_thickness_inflation_research.md)跨 10 月 3–4 日提出逐帧记录、离线联合优化、概率射线重放与成对地图导出；该研究并不证明 HBA、Global-LVBA 或 GLIM 已成功改善本机地图。

### 2026-10-04：离线地图、稠密对照与 FAST-LIO2 接入

- **离线概率候选图。** [实施与 A/B](../logs/26-10-4/offline_wall_validation_report.md)记录存档 `42551e8`、实验分支 `experiment/fastlivo_offline_wall_map`，实现完整 IMU 系扫描/同时间位姿记录、真实 LiDAR 原点射线重放、2 cm 三维概率体素和 5 cm 高度图。三轮新图墙带约 11.83 cm，旧在线最终约 13.66–14.34 cm；通道内侧误占并非每轮改善，不能声称漂移消失。HBA CLI 已构建并试跑，首轮墙带变差，因此默认使用原轨迹重放，HBA 保留显式对照。17 个 pytest 加 5 个包装记录通过，偏移起点旧图短程导航成功；在线预览和原保存服务仍是布尔累计图。
- **SLAM 与轨迹基准研究。** [SLAM Toolbox 比较](../logs/26-10-4/fastlivo_vs_slam_toolbox.md)区分融合里程计、概率图重放与自动回环/历史全局校正；[轨迹与基准接口核对](../logs/26-10-4/trajectory_benchmark_reference.md)明确 small_gicp `T_target_source`、射线原点和 SLAM Toolbox 2.8.5 旁路配置/保存陷阱。两份记录是源码研究，没有同数据 SLAM Toolbox 性能 A/B 或完整全局回环验收。
- **高度与图片实验。** [车高调查](../logs/26-10-4/gazebo_vehicle_height.md)及[快照/浏览器投影](../logs/26-10-4/lidar_snapshot_height_projection.md)核实当前 collision 顶高约 0.1485 m，提供独立 `lidarmap.pcd` 快照、地面基准和高度切片界面；14 个 pytest 加 2 个包装结果通过，实际浏览器生成/下载和快照哈希验证通过。[断点处理](../logs/26-10-4/projection_gap_processing.md)补入 165 个推断障碍格，保留原黑点及未知语义，19 个 pytest 加 2 个包装结果通过。图片没有自由空间证明、未切换为导航地图；原输入地面基准约偏差 20 cm，补点不能修正它。后续用户取消高度界面方向，转用既有二维地图。
- **稠密输出和上游对照。** [稠密输出](../logs/26-10-4/fastlivo_dense_output.md)开启后修复 VIO 清空临时扫描造成的空帧，使用 LIO 缓存完整扫描再按最终位姿投影；世界云每帧约 2403 → 21368 点，定位/累计保存仍用 10 cm 体素。[独立上游对照](../logs/26-10-4/upstream_dense_reference.md)隔离 `fast_livo_reference` 和构建目录，保留移植基线的四个算法/发布/保存函数，真实静止保存约 230 万原始 RGB 点。用户一轮退出竞态导致的新地图未保存、无法恢复；修复信号处理后新会话存盘成功，不能把新验证图冒充失败会话的恢复结果。
- **既有二维图 + FAST-LIVO2。** [静态图绑定验收](../logs/26-10-4/static_map_fastlivo_navigation.md)保持 `tb3_learning` 像素、尺寸和原点，将三维先验对齐并绑定；新增 4 个测试通过，当轮包汇总 32 条、2 条离线依赖测试跳过。独立域 226 偏移出生点导航成功、地图哈希不变、无 AMCL/建图节点；该组合显式关闭三维 collision cloud，局部障碍使用二维扫描。
- **战队开源方案核查。** [SCURM 调查](../logs/26-10-4/scurm_sentry_navigation_research.md)和[与默认 AMCL 链比较](../logs/26-10-4/scurm_vs_mini_nav_navigation_analysis.md)确认上游是 FAST-LIO2、Theta*、MPPI Omni 与恢复/比赛决策；当前接入定位不等于引入整套战队导航。[TDT 组件核验](../logs/26-10-4/tdt_navigation_open_source_research.md)及[固定版本比较](../logs/26-10-4/tdt_vs_mini_nav_navigation_analysis.md)分析 A*/动力学搜索与 MinimumSnap。普通 A* 小地图探针编译运行通过，完整示例因 OsqpEigen 缺失配置失败；没有双方导航性能排名。限定探针与失败记录见[证据说明](../logs/26-10-4/tdt_comparison_evidence/README.md)。
- **SCURM 部署与自研导航。** [建图部署](../logs/26-10-4/scurm_fastlio2_gazebo_deployment.md)完成三维 LiDAR/IMU FAST-LIO2 建图和短程运动，轮式相对位移差约 4.39 mm 只表示一致性。[先验定位接入](../logs/26-10-4/scurm_fastlio2_mini_nav_localization.md)采用 ICP 初始化与固定 ikd-tree 定位，发布动态 `map -> odom`、同时间匹配质量和 epoch，复用原二维地图、自研 Action 与 guard。往返导航、暂停前端约 0.4245 s 后定位失效/零速及恢复不续跑通过；加入终点转向迟滞以处理到达边界振荡，成功容差仍为 0.12 m。
- **人工初值、源码整理与直接入口。** [initialpose 接入](../logs/26-10-4/scurm_fastlio2_initialpose.md)替代自动初始化：未初始化拒绝目标，重新设置立即撤销任务并重启自身 ICP/FAST-LIO2，会话隔离；24 个 pytest 及隔离 Gazebo 验收通过。[源码整理](../logs/26-10-4/scurm_source_reorganization.md)归档 AMCL/FAST-LIO2 内核、ROS 接入和 `mini_nav_fastlio`，97 个实际用例通过并复验重置/到达；原算法和注释按哈希保留。[直接 launch](../logs/26-10-4/scurm_direct_launch.md)正式提供 `scurm_sim/fastlio2_navigation.launch.py`，设置域/分区与子进程环境，24 个 pytest 和独立无界面导航通过。默认 AMCL 保留，FAST-LIO2 入口不启动 LIVO2。

### 2026-10-05：膨胀对齐、统一碰撞几何与边界修复

- **膨胀核和默认参数分两轮对齐。** [同图差异研究](../logs/26-10-5/mini_nav_vs_nav2_inflation_and_motion.md)区分硬圈、软圈与控制策略；[膨胀核实现](../logs/26-10-5/inflation_alignment_implementation.md)对齐 Nav2 1.3.12 全图传播/代价/未知接收规则，默认地图 11536 格及随机地图逐格一致，80 个实际回归用例通过。[learning 默认对齐](../logs/26-10-5/inflation_defaults_learning_alignment.md)再设置显示内切半径 `0.22549849949589046 m`、外半径 `0.70 m`、衰减 `3.0`，83 个用例通过；真实车体 `0.24 + 0.02 m` 安全约束独立保留。此阶段曾让跟踪器读取双安全膨胀图，后续统一几何阶段改为原始碰撞快照。
- **现场碰撞误停诊断与方案。** [冻结现场诊断](../logs/26-10-5/collision_risk_live_diagnosis.md)复现中心格允许、连续车体拒绝，以及动态端点扩大成整个栅格和旧候选反复重试。[SCURM 碰撞/恢复源码核查](../logs/26-10-5/scurm_collision_handling_source_review.md)指出多候选/地图分工可借鉴，Omni 和自定义恢复不能原样复制；[修复方案](../logs/26-10-5/collision_risk_solution_proposal.md)提出统一几何、连续端点、静态与稳定动态分工和有界恢复。
- **统一几何与多候选实施。** [实施评估](../logs/26-10-5/collision_risk_implementation_assessment.md)完成 `CollisionGeometry`、原子 `CollisionMap`、`static_then_stable`、合法差分候选及重规划前取消旧 FollowPath；受阻/总期限不随重试清零，未启用自动倒车或初始重叠脱离。实测约 4 cm 定位误差促使增加显式 map 定位预算 0.03 m。93 个实际用例通过；隔离 Gazebo 四段静态路线和真实箱体绕行成功，前端冻结约 0.411 s 后零速、停车位移约 0.0313 m，独立真值几何审计通过。该轮 map 动态端点 0.32 m 判据后来在 10 月 6 日按观测来源修正。
- **安全终点与停车圆盘。** [到达后起点拒绝评估](../logs/26-10-5/start_rejection_after_path_assessment.md)修复规划/控制快照端点遗漏和首轮终点只查静态几何，增加覆盖 0.12 m 停车圆盘的终点净空；96 个实际用例通过。第一轮仍失败并保留证据，第二轮原五目标 5/5 到达、5/5 停稳后规划成功、0 次 `collision_risk`；到达指实际安全终点，可能在原目标 0.5 m 容差内替代。
- **未知状态与安全起点连接。** [显示/起点修复](../logs/26-10-5/unknown_state_stop_assessment.md)将 `avoiding_obstacle` 显示为“正在避障”，并允许实际安全位置连接多个安全邻接中心，避免所属中心不安全导致无路。100 个实际用例通过；冻结现场、无底盘真实规划节点、真实 RViz 插件和隔离 Gazebo 五次完整任务通过。所有阶段保持真实车体和未知禁行约束，不宣称有限回归覆盖了后续所有复发。

### 2026-10-06：实时激光与静态地图误差预算分离

[复发诊断与验收](../logs/26-10-6/recurrent_failure_assessment.md)先核验原进程确为上轮新版，再复现同一实时点在 map 判据下需 0.32 m、在 odom 下只需 0.29 m 的冲突；冻结最近点距车约 0.31147 m，静态格安全，却被多加的 map 定位预算拒绝。本次不是简单把问题归因于未重启。

`CollisionMap.points_frame_id` 明确端点观测帧。实际 LaserScan 端点按扫描时刻投影并保留在 `odom`，规划/平滑/控制将轨迹变换到该帧检查；静态地图继续在 `map` 按格面积和绝对定位预算检查。同帧 map 端点兼容路径仍保留原保守预算，未知帧、非法点、TF/输入失效继续拒绝。

| 当前几何来源 | 基础检查距离 | 预算含义 |
|---|---:|---|
| map 静态格、边界与未知格 | 0.29 m | 车体 0.24 + 安全 0.02 + 地图定位 0.03 |
| odom 原始实时 LaserScan 端点 | 0.29 m | 车体 0.24 + 安全 0.02 + 观测 0.03 |
| 原 map 同帧端点兼容输入 | 0.32 m | 保留原定位和观测两项预算 |

终点另加 0.12 m 停车圆盘约束，显示膨胀值保持 10 月 5 日 learning 对齐结果。该预算针对当前仿真，观测 0.03 m 来源于 1 cm Gaussian 噪声的三倍标准差，并非绝对噪声界。

四包构建及 13 个当前注册 CTest 目标通过，共 **103 个实际用例**。冻结完整地图得到安全 12 格路径；无底盘真实规划/控制节点成功，注入真正危险端点仍 `collision_risk` 并输出零速。独立域 228 Gazebo 复建物理位置后 **7 次任务及 7 次停车后规划全部成功**。2181 个真值样本的保守几何审计中，中心到柱/墙表面的距离下界分别为 0.326228 / 0.504675 m，均大于 0.26 m；最大定位误差 0.037688 m。测试环境已清理，用户现场保留；没有复现用户五小时运行的全部历史，也没有证明任意障碍和误差下都能通行。

## 当前边界

当前已具备低速基础单目标导航，规划和执行仍分别承担职责：`/mini_nav/global_path` 是几何参考与显示，不能绕过 FollowPath、定位/观测门控和 guard 直接作为运动许可。

- 尚无全部 Nav2 行为树、动态控制器插件、多目标巡航/对接、统一全栈 Lifecycle cleanup、实时调度或硬件急停；已有有限候选避障不等于完整 MPPI 或自动碰撞脱离。
- AMCL KLD 仍为简化实现，beam-skip/位姿持久化等兼容参数不等于已生效；长期真值误差、收敛、错误但自信的定位和搬移恢复仍缺少全面量化验收。
- FAST-LIVO2 / FAST-LIO2 旧图定位要求合格三维先验与人工邻域初值；未知位置全局搜索、重复走廊歧义、绑架恢复、实车时序/标定、坡道/多层/负障碍尚未验收。不能把近邻残差或轮式一致性当作绝对定位精度。
- 在线 FAST-LIVO 高度图仍是布尔累积预览；离线概率重放改善了部分墙带指标，尚未证明全部通道净宽和长程漂移解决。日志没有完整自动回环/历史全局校正的通过记录；HBA 首轮未改善，不能作为默认成功后端。
- 独立 guard 能处理命令/任务失联，不保证自身、DDS、bridge、系统或驱动故障时机械停车。已测有限仿真停车位移和指令响应，实车仍需驱动超时、制动测量与硬件保护。
- 系统 RViz 完整数据窗口退出异常仍未闭合；Wayland/XWayland 黑图不算截图验收。已有真实插件和部分浏览器测试通过，不能推断所有 GUI 和真实手柄面板体验已验收。
- 10 月 5–6 日成功路线对应 SCURM/FAST-LIO2 平地 Waffle 与指定地图；没有据此完成默认 AMCL、FAST-LIVO2、全部地图/动态场景的同等运动验收和长期成功率统计。

## 下一里程碑：重复路线与跨后端安全验收

下一步应在保留自研核心和当前安全预算的基础上，把最新修复纳入可重复系统验收：

1. 固定同一地图、底盘、目标序列和判定口径，记录原目标、实际终点、逐帧候选、TF/scan 和独立真值，重复验证连续任务、窄通道、柱边与临时/持续阻挡。
2. 分别在 AMCL、FAST-LIVO2 和 FAST-LIO2 入口验证重定位、取消/抢占、扫描/TF/时钟失效及进程冻结，报告失败原因、停车时延/距离、最小净空和定位误差。
3. 对地图实验分别验收在线预览、离线候选和静态图绑定；固定参考墙与通道净宽，保留低障碍/未知，不用缩小真实车体预算掩盖地图误占。
4. 使用同一底盘与运动约束做 Nav2 对照，区分膨胀逐格等价与驾驶性能；补齐长时间统计后再考虑速度、轨迹优化或更复杂恢复。

### 接口与坐标系约束

- 固定使用 Waffle，以对齐现有官方对照案例和机器人尺寸。
- Waffle 的 `/cmd_vel` 消息类型是 `geometry_msgs/msg/TwistStamped`；原始命令经唯一速度 guard 转发。
- 仿真阶段使用真实的 `odom -> base_*` 变换。
- 不发布静态 `map -> odom`。由当前选择的 AMCL、FAST-LIVO2 适配器或 FAST-LIO2 适配器动态估计，同一运行图中只有一个来源。
- 实时扫描端点保留扫描时间和观测帧；地图定位误差用于静态先验关系，不重复加入同一次相对激光几何。

## 后续学习顺序

1. **Astar 可视化**（已完成）：静态地图、选点和全局路径。
2. **仿真接口基座**（已完成）：Waffle、时钟、TF、里程计、激光与速度接口。
3. **定位与坐标系**（三种定位方向已有有限验收）：继续量化定位误差、收敛、重新初始化和失位恢复。
4. **路径跟踪与安全停止**（基础单目标闭环已完成）：继续验证跨后端故障停车与实际底盘动力学。
5. **代价地图与障碍处理**（滚动观测、候选避障、重规划和连续几何已接入）：扩展窄道、长期动态环境及误差边界验收。
6. **官方 Nav2 对照**（膨胀核和 learning 默认参数已逐格验证）：继续同底盘控制/恢复性能 A/B，不以色圈一致推断导航性能一致。
7. **地图与轨迹实验**：在可重放会话上评估全局历史校正、通道净宽和带时间轨迹，不将设计报告直接计为实施。

## 验收记录方式

每完成一个里程碑，在本文件补充：完成日期、启动命令、验证过的话题/TF、测试结果、已知限制，以及下一步的唯一目标。


## 早期记录补充（2026-09-30）：提高速度与不可达目标容差

本次在已有路径跟踪器上调整配置，并扩展自研 A* 的目标选择；保持原有安全检查。
本段保留当轮配置与验证；随后同日闭环实施及 10 月 5–6 日安全修复见上面的全日期记录。

- `path_follower.yaml`：线速度上限从 0.10 提高到 0.15 m/s，角速度上限从
  0.40 提高到 0.55 rad/s，接近目标、转弯减速和安全停车逻辑保持有效。
- `planning.goal_tolerance` 默认 0.5 m，0 禁用。参考官方 Jazzy
  [NavFnPlanner::makePlan](https://github.com/ros-navigation/navigation2/blob/f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501/nav2_navfn_planner/src/navfn_planner.cpp#L274-L310)，
  先规划原目标；原目标不可达时选容差半径内最近、从起点实际可达的安全格。
  使用目标格中心和欧氏距离，保留用户朝向，以替代终点判断到达；仍拒绝越界目标。
- 硬代价格、未知格、斜向穿角与连续车体扫掠规则均未放宽；容差内无路则清空路径。

验证命令（均在工作区根目录，先加载 Jazzy，构建后加载 overlay）：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup
source install/setup.bash
colcon test --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup --event-handlers console_cohesion+
colcon test-result --verbose
```

三个包构建通过。本次实际执行 7 个 CTest 目标、45 个 gtest 用例，全部通过，
其中 A* 共 10 个用例（本次新增 9 个），包含替代终点被跟踪器判定到达并停车的联动验证。
bringup 是资源包，无测试用例。`colcon test-result` 汇总显示 101 项、0 失败；
该汇总包含历史残留 XML，本次执行数采用当前 CTest 清单及对应新 XML，不能视为本次执行了 101 项。

隔离 ROS 节点验证使用域 208/209，并将控制器 `/cmd_vel` 重映射到测试话题，
未连接真实底盘或 Gazebo：障碍目标被替代为偏移 0.3606 m 的安全终点且朝向保留，
原目标可达时不替换，容差内无安全点时发布空路径；实际输出达到 0.15 m/s、
0.55 rad/s，局部障碍仍触发零速度与 `collision_risk`。
`tb3_learning` 新地图（112×103）也通过硬安全区目标替换，样例偏移 0.05 m，
单次选点到路径发布约 0.002 s；该数值仅是本次样例耗时，不是普遍性能保证。

尚未验证：新速度下 Gazebo 连续行驶的到达率、定位误差和转弯表现。
配置在启动时读取，需重启导航入口使其生效。


## 早期记录补充（2026-09-30）：修复接近终点时的 collision_risk 误停

跟踪器改为先计算候选速度，再对最多 1 秒且不超过前视距离的实际运动做连续碰撞检查。
每步不超过 0.05 秒，并计入圆弧到弦的偏离上界；原地转向保留当前位置检查。
车体硬安全半径、目标容差、限速及失效停车规则保留。

三个包构建成功，当前实际执行的 51 个 gtest 用例全部通过。
修复前 84/84 帧真实快照误停，修复后同一输入为 0/84。
实际 ROS 跟踪节点在隔离域中接入采集地图、路径及运动学反馈，完成到达和零速停车；
真实运动方向的障碍仍触发停车。
自动重启原进程的操作被审批阻止，未执行。随后只读核实后台已更换为修复版实例，
当前一轮 Gazebo 导航达到 `goal_reached` 并零速停车：位置误差 0.1133 m，朝向误差 0.1331 rad。
这一轮目标与原卡住目标不同，原位置以快照回归和隔离闭环验证为准；尚未完成长期固定路线验收。
详情见 `logs/26-9-30/collision_risk_fix_report.md`。


## 早期记录补充（2026-09-30）：RViz 常驻导航状态面板

新增独立包 `mini_nav_rviz_plugins`，通过 RViz Panel/pluginlib 提供中文只读监控。
`localization_astar.rviz` 默认在右侧加载“导航状态”，显示控制器状态及原因、
指令线/角速度、局部感知和连接超时；原有显示层、话题和相机视角保留。
控制器消息超过 2 秒无更新时显示失联，速度和局部感知超过 1 秒显示超时。
两个包构建通过，1 个真实 RViz/ROS 集成用例通过；bringup 无测试用例。
实际后台采样显示到达、零速、感知有效，右侧停靠可见。
临时完整窗口验收发现独立的 DDS 动态库卸载崩溃，修改前配置亦复现；
隔离面板测试正常退出，未修改工程 RMW 设置，完整窗口正常退出验收仍有限制。
报告及截图：`logs/26-9-30/rviz_navigation_status_panel.md`。
后续导航入口自动加载面板，当前旧 RViz 需重新加载工作区环境并重开。


## 早期记录补充（2026-09-30）：完整导航入口内置环境

`mini_localization_astar.launch.py` 默认设置域 61、Gazebo 分区 `mini_nav`、
Cyclone DDS 和 Waffle，无需在完整启动命令里逐项 export。
新增 `ros_domain_id`、`gz_partition`、`rmw_implementation` 参数用于覆盖默认值。
环境动作在包含仿真入口和创建节点之前执行，原有导航节点、地图和 RViz 配置保留。
资源包构建、Python 语法、安装入口 `--show-args` 和真实子进程环境继承验证通过；
bringup 无测试用例，本轮未重启模拟导航或重复算法测试。
其他终端观察话题仍须使用同一 ROS 域/RMW。
启动命令与证据见 `logs/26-9-30/launch_environment_update.md`。
