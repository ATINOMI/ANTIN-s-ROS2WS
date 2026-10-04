# mini_nav 项目状态

最后更新：2026-10-04
目标平台：ROS 2 Jazzy

`mini_nav` 是用于逐步理解移动机器人导航链路的自研学习项目。它以小而可验证的模块推进，并在后期与 `nav2_learning` 中的官方 Nav2 案例对照；`nav2_learning` 不属于本项目的运行依赖。

## 文档入口

- [架构说明](architecture.md)：模块职责、依赖方向、ROS 接口、TF 和启动链路。
- [贡献指南](contributing.md)：修改位置、代码风格、测试命令、运行验收和提交检查清单。
- [官方 AMCL 代码导读](amcl_code_walkthrough.md)：从节点启动、地图和初始位姿，到粒子更新、激光模型和 `map -> odom` 发布的源码承接关系。
- [规划代价地图](planning_costmap.md)：障碍膨胀、安全余量、A* 代价与 RViz 显示。

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
- `mini_nav_nodes`
  - 从 map_server 的 `/map` 接收 `nav_msgs/msg/OccupancyGrid`，转换为 `Costmap2D`。
  - 通过 `map_file` 参数直接加载标准 trinary YAML/PGM 地图，并发布 `/mini_nav/map`。
  - 收到 RViz `/goal_pose` 时，从机器人当前 `map -> base_footprint` TF 获取规划起点；`/initialpose` 使旧路径失效。
  - 将 A* 结果发布为 `/mini_nav/global_path`。
  - 独立发布 `/mini_nav/planning_costmap`，不改变原始地图和 AMCL 输入。
  - `AmclNode` 生命周期节点及 `/amcl_pose`、`/particle_cloud`、`map -> odom` 适配。
  - AMCL 定位更新与 TF 发布解耦：未达运动阈值时，仍按激光时间戳重发缓存的 `map -> odom`。
- 可视化
  - 已有 RViz 配置可显示静态地图、全局路径和地图坐标轴。

## 本次完成记录

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

## 当前边界

当前 A* 可以接收 map_server 提供的静态 `/map` 并进行规划，AMCL 基础定位链路已经打通，但仍是静态地图上的规划与可视化演示，尚未构成可移动机器人的完整导航闭环。以下能力尚未接入：

- Gazebo 长时间运动下的 AMCL 粒子收敛指标、定位误差和丢失恢复验证。
- 路径跟踪、碰撞规避、恢复行为和 `/cmd_vel` 速度控制。
- 局部图的障碍更新已独立验收；尚未接入路径执行时的局部避障和安全停车。

因此，当前阶段不能把 `/mini_nav/global_path` 视为机器人可执行轨迹。

原 `maps/turtlebot3_map.yaml` 已接入官方 map_server 和自研 AMCL，初始位姿、激光、TF 和 `map -> odom` 基础链路已验证，低速 teleop 下的 TF 连续刷新也已通过；仍需用长时间和可量化的运动轨迹评估定位误差、收敛速度和丢失恢复能力。

## 下一里程碑：路径跟踪与安全停止

下一步利用已经建立的 `map -> odom -> base_*` 链路实现路径跟踪，不直接接入 Nav2 控制器：

1. 在已验证的 `map -> odom -> base_*` 链路上接入路径跟踪。
2. 将 `/mini_nav/global_path` 转换为受限 `/cmd_vel`，先实现低速路径跟踪。
3. 加入目标到达、超时和零速度安全停止。
4. 保持 `astar_demo` 与仿真、定位入口边界清晰：路径规划和路径执行仍然是两个可独立验证的模块。

### 接口与坐标系约束

- 固定使用 Waffle，以对齐现有官方对照案例和机器人尺寸。
- Waffle 的 `/cmd_vel` 消息类型是 `geometry_msgs/msg/TwistStamped`；后续控制器必须从该接口开始设计。
- 仿真阶段使用真实的 `odom -> base_*` 变换。
- 不发布静态 `map -> odom`。该变换由 AMCL 根据地图、激光和里程计估计产生，避免引入错误的地图坐标语义。

## 后续学习顺序

1. **Astar 可视化**（已完成）：静态地图、选点和全局路径。
2. **仿真接口基座**（已完成）：Waffle、时钟、TF、里程计、激光与速度接口。
3. **定位与坐标系**（官方入口、自研基础链路和 TF 平滑刷新已验证）：继续量化评估 `map -> odom -> base_*` 链路下的定位误差、收敛和丢失恢复。
4. **路径跟踪与安全停止**：将路径转换为受限速度命令，并加入目标到达、超时和零速度保护。
5. **代价地图与障碍处理**（局部图构建与 Gazebo 验收已完成）：后续接入局部避障和安全停车。
6. **官方 Nav2 对照**：将自研模块与 `nav2_learning` 的 AMCL、规划、控制和恢复行为逐项比较。

## 验收记录方式

每完成一个里程碑，在本文件补充：完成日期、启动命令、验证过的话题/TF、测试结果、已知限制，以及下一步的唯一目标。


## 2026-09-30：提高速度与不可达目标容差

本次在已有路径跟踪器上调整配置，并扩展自研 A* 的目标选择；保持原有安全检查。
当前完整能力与缺口以 `logs/26-9-30/navigation_gap_research.md` 的代码核查为准，
上文早期阶段描述不代表当前路径跟踪器尚未实现。

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


## 2026-09-30：修复接近终点时的 collision_risk 误停

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


## 2026-09-30：RViz 常驻导航状态面板

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


## 2026-09-30：完整导航入口内置环境

`mini_localization_astar.launch.py` 默认设置域 61、Gazebo 分区 `mini_nav`、
Cyclone DDS 和 Waffle，无需在完整启动命令里逐项 export。
新增 `ros_domain_id`、`gz_partition`、`rmw_implementation` 参数用于覆盖默认值。
环境动作在包含仿真入口和创建节点之前执行，原有导航节点、地图和 RViz 配置保留。
资源包构建、Python 语法、安装入口 `--show-args` 和真实子进程环境继承验证通过；
bringup 无测试用例，本轮未重启模拟导航或重复算法测试。
其他终端观察话题仍须使用同一 ROS 域/RMW。
启动命令与证据见 `logs/26-9-30/launch_environment_update.md`。
