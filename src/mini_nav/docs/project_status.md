# mini_nav 项目状态

最后更新：2026-08-30
目标平台：ROS 2 Jazzy

`mini_nav` 是用于逐步理解移动机器人导航链路的自研学习项目。它以小而可验证的模块推进，并在后期与 `nav2_learning` 中的官方 Nav2 案例对照；`nav2_learning` 不属于本项目的运行依赖。

## 文档入口

- [架构说明](architecture.md)：模块职责、依赖方向、ROS 接口、TF 和启动链路。
- [贡献指南](contributing.md)：修改位置、代码风格、测试命令、运行验收和提交检查清单。
- [官方 AMCL 代码导读](amcl_code_walkthrough.md)：从节点启动、地图和初始位姿，到粒子更新、激光模型和 `map -> odom` 发布的源码承接关系。

## 当前已完成

- `mini_nav_core`
  - 二维静态代价地图 `Costmap2D`。
  - 栅格坐标与世界坐标之间的转换。
  - 四邻域 A* 全局路径规划。
  - 代价地图与 A* 规划器的单元测试。
  - `localization/` 中的 `LocalizationMap`、差分运动模型、激光模型、`KdTree` 和 `ParticleFilter`。
- `mini_nav_nodes`
  - 从 map_server 的 `/map` 接收 `nav_msgs/msg/OccupancyGrid`，转换为 `Costmap2D`。
  - 通过 `map_file` 参数直接加载标准 trinary YAML/PGM 地图，并发布 `/mini_nav/map`。
  - 通过 RViz 的 `/initialpose` 和 `/goal_pose` 接收起点、终点。
  - 将 A* 结果发布为 `/mini_nav/global_path`。
  - `AmclNode` 生命周期节点及 `/amcl_pose`、`/particle_cloud`、`map -> odom` 适配。
- 可视化
  - 已有 RViz 配置可显示静态地图、全局路径和地图坐标轴。

## 本次完成记录

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

### M1：自研定位核心（2026-08-27）

- 在 `mini_nav_core` 增加 ROS 无关的 `localization/` 子模块。
- 完成 `Pose2D`、`Covariance3`、`Particle`、`PoseEstimate` 数据结构。
- 完成粒子初始化、完整协方差 Cholesky 采样、权重校验/归一化、系统重采样、圆周角度均值和协方差估计。
- 新增 9 项粒子滤波单元测试；与原有地图/A* 测试一起通过，测试结果为 26 项、0 错误、0 失败、0 跳过。
- 该记录描述 2026-08-27 的核心阶段；当前已继续完成差分运动模型、激光模型、`KdTree`、ROS 适配节点和自研 launch 接入。

## 当前边界

当前 A* 可以接收 map_server 提供的静态 `/map` 并进行规划，AMCL 基础定位链路已经打通，但仍是静态地图上的规划与可视化演示，尚未构成可移动机器人的完整导航闭环。以下能力尚未接入：

- Gazebo 长时间运动下的 AMCL 粒子收敛指标、定位误差和丢失恢复验证。
- 路径跟踪、碰撞规避、恢复行为和 `/cmd_vel` 速度控制。
- 动态代价地图更新、动态障碍处理和局部避障。

因此，当前阶段不能把 `/mini_nav/global_path` 视为机器人可执行轨迹。

现有 `maps/turtlebot3_map.yaml` 已接入官方 map_server 和自研 AMCL，初始位姿、激光、TF 和 `map -> odom` 基础链路已验证；仍需通过长时间运动确认地图、激光和坐标原点在动态过程中的一致性。

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
3. **定位与坐标系**（官方入口已验证，自研基础链路已验证）：理解并继续评估合法的 `map -> odom -> base_*` 链路。
4. **路径跟踪与安全停止**：将路径转换为受限速度命令，并加入目标到达、超时和零速度保护。
5. **代价地图与障碍处理**：利用激光数据构建局部代价地图。
6. **官方 Nav2 对照**：将自研模块与 `nav2_learning` 的 AMCL、规划、控制和恢复行为逐项比较。

## 验收记录方式

每完成一个里程碑，在本文件补充：完成日期、启动命令、验证过的话题/TF、测试结果、已知限制，以及下一步的唯一目标。
