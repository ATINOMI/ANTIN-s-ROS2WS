# mini_nav 架构说明

最后更新：2026-08-31

## 1. 项目定位

`mini_nav` 是一个用于学习移动机器人导航链路的 ROS 2 Jazzy 项目。它把算法实现、ROS 适配和运行编排拆成三个独立模块，并使用官方 Nav2 的地图服务与 AMCL 作为对照基础。

当前项目已经可以完成：

- 从静态地图构建内部 `Costmap2D`。
- 使用自研四邻域 A* 生成全局路径。
- 在 Gazebo Waffle 仿真中运行官方或自研 map_server、AMCL、传感器和 TF。
- 在 RViz 中设置初始位姿和目标点并显示路径。

当前项目还不能完成：

- 路径跟踪和 `/cmd_vel` 控制。
- 动态障碍更新、局部避障和恢复行为。
- 把 `/mini_nav/global_path` 作为机器人可直接执行的轨迹。

## 2. 顶层目录

`src/mini_nav/` 本身不是 ROS 包，而是三个 ROS 包和项目资源的集合。下面列出当前需要维护的具体文件；`build/`、`install/`、`log/` 等生成目录不属于源码架构。

~~~text
src/mini_nav/                                      # mini_nav 项目集合根目录，本身不是 ROS 包
├── mini_nav_core/                                 # ROS 无关的地图、A* 与 AMCL 算法包
│   ├── include/mini_nav_core/                     # mini_nav_core 对外公开的 C++ 头文件
│   │   ├── localization/                          # 自研 AMCL 的 ROS 无关算法接口
│   │   │   ├── types.hpp                         # 位姿、协方差、粒子和定位估计数据类型
│   │   │   ├── laser_scan_data.hpp               # 激光扫描的 ROS 无关数据结构
│   │   │   ├── localization_map.hpp               # 定位地图、距离场、采样和射线查询接口
│   │   │   ├── motion_model.hpp                   # 粒子运动模型抽象接口
│   │   │   ├── differential_motion_model.hpp      # 差速里程计运动模型声明
│   │   │   ├── laser_model.hpp                    # 激光观测模型抽象接口
│   │   │   ├── likelihood_field_model.hpp         # 似然场激光模型声明
│   │   │   ├── beam_model.hpp                     # Beam 激光模型声明
│   │   │   ├── kd_tree.hpp                        # ParticleFilter 专用空间分箱与 KLD 统计接口
│   │   │   └── particle_filter.hpp                # 粒子滤波主流程及节点层调用接口
│   │   ├── navigator/                             # 全局路径规划算法接口
│   │   │   └── astar_navigator.hpp                # 四邻域 A* 规划器声明
│   │   └── map/                                   # 通用二维栅格地图接口
│   │       └── costmap_2d.hpp                     # Costmap2D 存储、坐标转换和绘图接口
│   ├── src/                                       # mini_nav_core 算法实现
│   │   ├── localization/                          # 自研 AMCL 算法实现
│   │   │   ├── localization_map.cpp               # 距离场、自由空间采样和射线投射实现
│   │   │   ├── differential_motion_model.cpp      # 差速运动分解、噪声采样和粒子预测实现
│   │   │   ├── likelihood_field_model.cpp         # 似然场权重计算实现
│   │   │   ├── beam_model.cpp                     # Beam 模型概率与预期量程实现
│   │   │   ├── kd_tree.cpp                        # 粒子空间分箱和占用 bin 统计实现
│   │   │   └── particle_filter.cpp                # 初始化、更新、归一化、重采样和估计实现
│   │   ├── navigator/                             # 全局路径规划算法实现
│   │   │   └── astar_navigator.cpp                # A* 搜索、代价记录和路径回溯实现
│   │   └── map/                                   # 二维栅格地图实现
│   │       └── costmap_2d.cpp                     # 栅格存储、边界检查和坐标转换实现
│   ├── test/                                      # mini_nav_core 的 ROS 无关单元测试
│   │   ├── test_astar_planner.cpp                 # A* 绕障和路径生成测试
│   │   ├── test_costmap_2d.cpp                    # 地图边界、坐标转换和绘图测试
│   │   └── test_particle_filter.cpp               # 粒子初始化、权重、重采样和估计测试
│   ├── CMakeLists.txt                             # core 库、依赖、安装和测试构建规则
│   └── package.xml                                # mini_nav_core ROS 包清单与依赖声明
├── mini_nav_nodes/                                # ROS 消息、TF、生命周期和算法适配节点包
│   ├── include/                                   # 节点类的公开头文件
│   │   ├── costmap_publisher.hpp                  # 地图加载、A* 请求和可视化节点声明
│   │   └── mini_nav_nodes/                        # 使用包命名空间组织的节点头文件
│   │       └── amcl_node.hpp                      # 自研 AMCL 生命周期节点与 TF 缓存声明
│   ├── src/                                       # ROS 节点实现与进程入口
│   │   ├── costmap_publisher.cpp                  # 地图转换、交互选点和路径发布实现
│   │   ├── amcl_node.cpp                          # AMCL 生命周期、消息适配、滤波调度和 TF 发布实现
│   │   └── main.cpp                               # 初始化、构造节点、spin 和 shutdown 的统一入口
│   ├── test/                                      # mini_nav_nodes 的节点层单元测试
│   │   └── test_amcl_tf_cache.cpp                 # TF 缓存、重时间戳和失效行为测试
│   ├── CMakeLists.txt                             # 两个节点可执行文件及节点测试的构建规则
│   └── package.xml                                # mini_nav_nodes ROS 包清单与依赖声明
├── mini_nav_bringup/                              # 仿真、地图、参数和节点组合的运行编排包
│   ├── launch/                                    # ROS 2 启动入口
│   │   ├── official_localization_astar.launch.py  # 官方 AMCL 对照组完整入口
│   │   ├── mini_localization_astar.launch.py      # 自研 AMCL 被测组完整入口
│   │   ├── waffle_sim.launch.py                   # 官方 TurtleBot3 Waffle Gazebo 仿真入口
│   │   └── sim_check.launch.py                    # 仅启动 RViz 检查现有仿真与 TF
│   ├── config/                                    # 运行参数文件
│   │   └── amcl_waffle.yaml                       # 官方与自研 AMCL 共用的 Waffle 对照参数
│   ├── maps/                                      # map_server 使用的静态地图资源
│   │   ├── turtlebot3_map.yaml                    # 地图图像路径、分辨率、原点和阈值元数据
│   │   └── turtlebot3_map.pgm                     # 官方示例环境的占用栅格图像
│   ├── CMakeLists.txt                             # launch、config、maps 和 RViz 的安装规则
│   └── package.xml                                # bringup 包清单与运行依赖声明
├── rviz/                                          # 项目级 RViz 显示配置
│   ├── astar_navigation.rviz                      # 静态地图与 A* 路径演示布局
│   └── localization_astar.rviz                    # 定位粒子、机器人、激光和路径联合布局
├── scripts/                                       # 项目环境和便捷启动脚本
│   ├── env_mini_nav.sh                            # 设置 ROS_DOMAIN_ID、GZ_PARTITION 和模型环境
│   └── start_astar_navigation.sh                  # 不启动仿真和 AMCL 的独立 A* 演示脚本
├── reference/                                     # 只用于阅读对照、不作为生产依赖的参考源码
│   └── nav2_amcl_jazzy/                           # Jazzy 官方 Nav2 AMCL 源码快照
│       ├── include/nav2_amcl/                     # 官方 AMCL 头文件
│       │   ├── map/                               # 官方地图与距离空间接口
│       │   ├── motion_model/                      # 官方运动模型接口
│       │   ├── pf/                                # 官方粒子滤波与 KD-tree 接口
│       │   └── sensors/laser/                     # 官方激光传感器模型接口
│       ├── src/amcl_node.cpp                      # 官方生命周期节点和 ROS 适配实现
│       ├── src/main.cpp                           # 官方 AMCL 进程入口
│       ├── src/map/                               # 官方地图、距离场和射线实现
│       ├── src/motion_model/                      # 官方差速与全向运动模型实现
│       ├── src/pf/                                # 官方粒子滤波、KD-tree 和概率工具实现
│       ├── src/sensors/laser/                     # 官方激光观测模型实现
│       ├── COLCON_IGNORE                          # 阻止参考快照参与当前工作区构建
│       ├── README.md                              # 上游源码包说明
│       └── REFERENCE.md                           # 快照来源、版本和使用边界说明
└── docs/                                          # mini_nav 设计、调研、状态和协作文档
    ├── architecture.md                            # 当前架构、目录、接口、数据流和启动边界
    ├── amcl_code_walkthrough.md                   # 官方 AMCL 源码调用链导读
    ├── amcl_research.md                           # AMCL 算法与模块拆分调研
    ├── contributing.md                            # 构建、测试、修改和提交规范
    ├── project_status.md                          # 已完成里程碑、验收结果和当前限制
    └── technicial-detail-elaboration.md           # 细节推导和补充实现说明
~~~

### 2.1 `mini_nav_core`：算法模块

- `navigator/astar_navigator.hpp` 是 A* 的外部接缝：调用者只需提供 `Costmap2D`、起点和终点。
- `navigator/astar_navigator.cpp` 隐藏 open list、代价记录、父节点回溯和无路径处理。
- `map/costmap_2d.hpp/.cpp` 隐藏栅格内存、世界坐标转换、越界检查和地图绘制。
- `localization/` 提供 `LocalizationMap`、差分运动模型、激光模型、`PoseBinIndex` 和 `ParticleFilter`；定位算法不依赖 ROS。
- `test/` 只通过核心模块接口测试行为，不依赖 ROS 节点、Gazebo 或 RViz。

### 2.1.1 `localization/`：AMCL 算法文件职责

`localization/` 是自研 AMCL 的 ROS 无关核心。它只接收普通 C++ 数据结构和 `Costmap2D`，不包含 `rclcpp`、ROS 消息或 TF 代码。这样 `ParticleFilter` 可以在单元测试中独立运行，`AmclNode` 只承担 ROS 到核心数据的适配。

| 文件 | 模块角色 | 主要职责和接口 |
|---|---|---|
| `types.hpp` | 核心数据模型 | 定义 `Pose2D`、`Covariance3`、`Particle` 和 `PoseEstimate`。统一使用米和弧度，并提供带范围检查的协方差访问、角度归一化和角距离计算。 |
| `laser_scan_data.hpp` | 激光数据模型 | 保存距离数组、角度起点、角度增量、最小量程和最大量程；它是 `sensor_msgs/msg/LaserScan` 到算法核心之间的轻量数据接缝。 |
| `localization_map.hpp/.cpp` | 定位地图模块 | 从 `Costmap2D` 构造只读地图快照，保存原点、分辨率和栅格代价；提供世界坐标到栅格查询、自由/障碍判断、障碍距离查询、自由栅格采样和射线投射。构造函数内部用优先队列传播障碍距离场，供似然场模型快速查询。 |
| `motion_model.hpp` | 运动模型接口 | 定义 `UpdateParticles()` 抽象接口。调用者只提供上一帧和当前帧里程计位姿，具体噪声模型由实现隐藏。 |
| `differential_motion_model.hpp/.cpp` | 差速运动模型 | 将里程计增量分解为第一次旋转、平移和第二次旋转，依据 `alpha1` 到 `alpha5` 为每个粒子注入高斯噪声，然后原地更新粒子位姿。 |
| `laser_model.hpp` | 激光模型接口 | 定义 `ApplyMeasurementLikelihood()` 抽象接口，使粒子滤波器不依赖某一种激光观测模型。 |
| `likelihood_field_model.hpp/.cpp` | 似然场模型 | 将有效激光束投影到地图，查询命中点到最近障碍物的距离，并按 `z_hit`、`z_rand` 和 `sigma_hit` 累积粒子权重；按 `max_beams` 降采样以控制计算量。 |
| `beam_model.hpp/.cpp` | Beam 模型 | 通过 `LocalizationMap::CastRay()` 计算地图预期量程，再组合命中、短测距、最大量程和随机测量概率。它与似然场模型共享 `LaserModel` 接口，目前由参数选择。 |
| `kd_tree.hpp/.cpp` | 粒子空间统计 | 按线性和角度分辨率把粒子量化到空间 bin，并统计占用 bin 数。当前实现使用哈希分箱，不是对外暴露的通用搜索树；公共接口只有 `BuildPoseBinIndex()`、兼容入口 `Build()` 和 `GetOccupiedBinCount()`，只服务 `ParticleFilter` 的 KLD 自适应粒子数计算。 |
| `particle_filter.hpp/.cpp` | AMCL 主算法模块 | 持有 `MotionModel`、`LaserModel` 和 `PoseBinIndex`，统一实现局部高斯初始化、全局自由栅格初始化、运动更新、传感器更新、权重归一化、系统重采样和位姿/协方差估计。它是算法核心对节点层的主要接缝。 |
| `test_particle_filter.cpp` | 核心测试 | 验证粒子数量、协方差采样、权重校验与归一化、重采样偏好、圆周角度均值和未初始化状态。 |

### 2.1.2 AMCL 核心调用顺序

定位核心的一次更新遵循固定顺序，ROS 节点不直接操作粒子：

```text
OccupancyGrid
     │  AmclNode 转换
     ▼
Costmap2D ──► LocalizationMap
                  │
/initialpose ──────┴──► ParticleFilter::InitializeLocalized
                              │
odom TF ───────────────► MotionModel::UpdateParticles
                              │
LaserScan + 激光 TF ───► LaserModel::ApplyMeasurementLikelihood
                              │
                              ├── NormalizeWeights
                              ├── PoseBinIndex 分箱 + Resample
                              └── Estimate
                                      │
                                      ├── /amcl_pose
                                      ├── /particle_cloud
                                      └── map -> odom
```

`ParticleFilter` 的接口是外部接缝：节点只负责按时序提供地图、里程计、扫描和传感器安装位姿；运动噪声、激光权重和粒子组织细节都留在核心实现内部。`PoseBinIndex` 不应被 `AmclNode` 或其他调用者直接使用。

### 2.2 `mini_nav_nodes`：ROS 适配模块

- `costmap_publisher.hpp` 声明节点的 ROS 参数、发布器、订阅器和内部地图状态。
- `costmap_publisher.cpp` 是主要实现：接收 `/map` 或读取 `map_file`，转换成原始图与膨胀规划图；收到 `/goal_pose` 时从 `map -> base_footprint` TF 取得当前起点，调用 `AStarPlanner`，发布 `/mini_nav/map`、`/mini_nav/planning_costmap` 和 `/mini_nav/global_path`。收到 `/initialpose` 时清除旧路径，等待定位 TF 更新。
- `amcl_node.cpp` 负责 `AmclNode` 的生命周期、消息、TF、服务和结果发布；`main.cpp` 只负责初始化 ROS、构造对应节点和 spin。
- 该包不负责 Gazebo、AMCL 或 RViz 启动；这些属于 `mini_nav_bringup`。

### 2.2.1 `mini_nav_nodes` 中的 AMCL 文件职责

| 文件 | 模块角色 | 主要职责 |
|---|---|---|
| `include/mini_nav_nodes/amcl_node.hpp` | ROS 适配节点接口 | 声明 `AmclNode` 生命周期回调、地图/初始位姿/激光回调、服务回调、TF 查询、消息转换和结果发布函数；保存 ROS 通信对象、生命周期发布器、TF 对象、最近一次有效的 `map -> odom` 缓存，以及 `mini_nav_core` 的 `LocalizationMap` 和 `ParticleFilter`。 |
| `src/amcl_node.cpp` | ROS 适配节点实现 | 负责参数声明与校验、生命周期资源管理、QoS、地图转换、MessageFilter 激光接入、初始位姿处理、ParticleFilter 调用、`/amcl_pose`、`/particle_cloud` 和 `map -> odom` 发布。粒子滤波未达到运动更新阈值时，它仍按激光帧时间戳重发缓存 TF；算法计算不在这里重新实现。 |
| `src/main.cpp` | 进程入口 | 只负责 `rclcpp::init()`、节点构造、spin 和 shutdown。通过 CMake 为不同目标设置的宏选择 `AmclNode` 或 `CostmapPublisherNode`，不加载地图、不绘制迷宫，也不把业务逻辑写入入口。 |
| `test/test_amcl_tf_cache.cpp` | 节点层 TF 单元测试 | 验证首次有效估计前不生成 TF、缓存几何值在重发时保持不变且时间戳向前刷新，以及状态重置后旧缓存不可复用。 |
| `CMakeLists.txt` | 构建接缝 | 将 `main.cpp + amcl_node.cpp` 编译为 `mini_nav_amcl_node`，将 `main.cpp + costmap_publisher.cpp` 编译为 `costmap_publisher_node`；同时链接 `mini_nav_core` 和 AMCL 所需的生命周期、消息、TF、服务和 MessageFilter 依赖，并在 `BUILD_TESTING` 下构建 `test_amcl_tf_cache`。 |
| `package.xml` | 依赖声明 | 声明 `mini_nav_core`、`rclcpp_lifecycle`、`sensor_msgs`、`nav2_msgs`、`tf2`、`tf2_ros`、`message_filters` 等构建和运行依赖，以及 `ament_cmake_gtest` 测试依赖。 |

### 2.2.2 `AmclNode` 的生命周期和运行职责

`AmclNode` 是 ROS 侧的适配器，状态变化由 `nav2_lifecycle_manager` 驱动：

1. **构造阶段**：声明 AMCL 和 Nav2 风格参数，但不创建地图、激光订阅或粒子滤波器。
2. **`on_configure()`**：读取并校验帧名、话题名、粒子数、激光模型和噪声参数；创建 `LocalizationMap`、运动模型、激光模型、`ParticleFilter`、TF Buffer/Listener/Broadcaster、订阅器、发布器和服务。
3. **回调组配置**：地图、初始位姿和激光输入使用同一个 `MutuallyExclusiveCallbackGroup`，并设置为自动加入节点 executor。`main.cpp` 使用普通 `rclcpp::spin()`，所以该回调组必须被 executor 调度；否则 ROS 图虽然存在订阅连接，回调不会执行。
4. **`on_activate()`**：激活 `/amcl_pose` 和 `/particle_cloud` 两个生命周期发布器，并允许激光回调进入滤波流程。
5. **地图回调**：校验地图 frame、原点旋转、尺寸和数据长度，把 `OccupancyGrid` 转成 `Costmap2D`，再构造 `LocalizationMap`。地图准备好后，等待 `/initialpose` 或初始化服务。
6. **初始位姿回调**：校验消息 frame 和地图自由空间，将 ROS 的 6×6 协方差提取为核心使用的 3×3 协方差，调用 `ParticleFilter::InitializeLocalized()`，并允许后续激光更新。当前配置使用 RViz 的 `/initialpose`；`set_initial_pose` 和 `initial_pose.*` 参数主要用于保持 Nav2 风格参数可加载，尚不代表节点启动时自动完成初始化。
7. **激光回调**：`MessageFilter` 等待 `base_scan -> odom` 在扫描时间戳可用；节点再查询 `odom -> base_footprint` 和 `base_footprint -> base_scan`。达到 `update_min_d` 或 `update_min_a` 时，调用运动模型和激光模型，完成权重归一化、重采样和位姿估计；未达阈值时不运行这组较重的计算，但仍用当前激光时间戳重发缓存的 `map -> odom`。
8. **输出阶段**：一次有效滤波更新会发布 `/amcl_pose` 和 `/particle_cloud`，并用估计的 `map -> base_footprint` 与里程计的 `odom -> base_footprint` 计算 `map -> odom`。该变换的几何值被缓存，发布时间戳是当前激光时间加 `transform_tolerance`。
9. **重置与清理阶段**：首次有效估计前不存在可发布的缓存。成功替换地图、处理新初始位姿、执行全局定位、`on_cleanup()` 或 `on_shutdown()` 时必须使旧 TF 缓存失效，防止新定位周期误用旧的坐标关系。`on_deactivate()` 停止处理并停用发布器；`on_cleanup()` 还会断开 MessageFilter 并释放订阅器、TF、地图和粒子滤波器。

#### TF 计算频率与发布频率分离

`update_min_d` 和 `update_min_a` 只决定何时运行粒子滤波更新，不应同时限制 TF 刷新。如果未达运动阈值就直接返回，最后一条 `map -> odom` 会逐渐超出 `transform_tolerance`，RViz 不能用它与持续更新的 `odom -> base_footprint` 组合，视觉上便会出现暂停后跳动。

节点将“定位估计是否更新”与“TF 是否持续可用”拆成两个状态：

```text
达到运动阈值
    └── ParticleFilter 更新
            └── 计算并缓存 map -> odom
                    └── 以 scan_stamp + transform_tolerance 发布

未达运动阈值，且缓存有效
    └── 保持变换几何值不变
            └── 以新的 scan_stamp + transform_tolerance 重发
```

因此 `/amcl_pose` 和 `/particle_cloud` 仍按定位计算节奏更新，`map -> odom` 则在每帧可处理的激光数据上保持时间新鲜。机器人在两次定位估计之间的连续运动由 `odom -> base_footprint` 表达，RViz 可以沿完整 TF 链平滑显示。

### 2.3 `mini_nav_bringup`：运行编排模块

`mini_nav_bringup` 只描述“启动哪些现成模块以及如何连接它们”，不实现地图、定位或规划算法。启动文件使用包共享目录查找资源，不依赖工作区绝对路径。

#### 2.3.1 启动文件职责

| 文件 | 入口类型 | 主要职责 |
|---|---|---|
| `launch/official_localization_astar.launch.py` | 官方对照入口 | 启动同一套 Waffle Gazebo 仿真，然后包含系统 `nav2_bringup/launch/localization_launch.py`，由官方 `nav2_amcl` 和官方 map server 提供定位；再启动自研 `costmap_publisher_node` 和项目 RViz。 |
| `launch/mini_localization_astar.launch.py` | 自研入口 | 启动同一套 Waffle Gazebo 仿真和官方 `nav2_map_server/map_server`，将 `mini_nav_nodes/mini_nav_amcl_node` 命名为 `amcl`，由 `nav2_lifecycle_manager` 管理 `map_server` 和自研 `amcl` 的 configure/activate；不启动 `nav2_amcl`。 |
| `launch/waffle_sim.launch.py` | 仿真适配入口 | 复用系统安装的 TurtleBot3 Waffle world，提供机器人、`/scan`、`/odom`、仿真 TF 和 `/clock`。它不启动地图服务、不启动 AMCL，也不发布 `map -> odom`。 |
| `launch/sim_check.launch.py` | RViz 检查入口 | 只打开 RViz 检查已有仿真和 TF 环境，不能代替地图服务、AMCL 或 A* 节点。 |

两个完整入口的可替换部分只有定位实现：

~~~text
official_localization_astar.launch.py
    └── nav2_bringup/localization_launch.py
            └── nav2_amcl + nav2_map_server

mini_localization_astar.launch.py
    └── nav2_map_server/map_server
        + mini_nav_nodes/mini_nav_amcl_node
        + nav2_lifecycle_manager
~~~

这样可以在相同地图、Gazebo 模型、初始仿真位置、RViz 配置和 A* 节点下比较官方 AMCL 与自研 AMCL。

#### 2.3.2 配置、地图和安装文件

| 文件 | 模块角色 | 主要职责 |
|---|---|---|
| `config/amcl_waffle.yaml` | AMCL 参数配置 | 保存帧名、激光模型、粒子数、运动噪声、更新阈值和 `use_sim_time` 之外的 AMCL 参数。参数采用 Nav2 风格命名，官方入口和自研入口共用以便对照。 |
| `maps/turtlebot3_map.yaml` | 地图元数据 | 指向同目录的 `turtlebot3_map.pgm`，声明分辨率、原点、占用阈值和地图模式；由官方 `map_server` 读取，不由 `main.cpp` 或 AMCL 绘制。 |
| `maps/turtlebot3_map.pgm` | 地图栅格 | 官方示例使用的静态地图图像，作为 map server 的输入；自研 AMCL 通过 `/map` 接收其转换后的 `OccupancyGrid`。 |
| `CMakeLists.txt` | 资源安装规则 | 安装 `launch/`、`config/`、`maps/`，并把上级 `mini_nav/rviz/` 安装到包共享目录的 `rviz/` 下；不编译算法代码。 |
| `package.xml` | bringup 运行依赖 | 声明 launch、map server、lifecycle manager、RViz 和仿真相关的运行依赖。 |

`map`、`params_file`、`use_sim_time`、`autostart`、`x_pose` 和 `y_pose` 是入口层参数。地图路径和参数文件默认值通过 `get_package_share_directory("mini_nav_bringup")` 解析，避免把当前机器的工作区路径写入启动接口。

### 2.4 项目级资源、参考源码和文档

#### 2.4.1 RViz 和脚本

| 文件 | 作用 |
|---|---|
| `rviz/astar_navigation.rviz` | 服务于不启动仿真和 AMCL 的静态 A* 演示，重点显示地图、目标点和 A* 路径。 |
| `rviz/localization_astar.rviz` | 服务于完整定位入口，显示地图、机器人模型、激光、AMCL 粒子云、定位姿态和 A* 路径。 |
| `scripts/env_mini_nav.sh` | 设置项目运行所需的 `ROS_DOMAIN_ID`、`GZ_PARTITION`、TurtleBot3 模型等环境。需要互相发现的终端都必须加载同一环境。 |
| `scripts/start_astar_navigation.sh` | 不启动 Gazebo 或 AMCL，直接给 `costmap_publisher_node` 传入地图文件，适合单独验证自研 A*。 |

工作区根目录的 `maps/` 和 `scripts/run_nav2_case.sh` 不属于 `mini_nav` 内部模块：前者是工作区共享地图，后者负责调度其他 Nav2 案例。项目中的 `*.orig` 文件是临时备份，不应被 CMake 安装或提交。

#### 2.4.2 `reference/nav2_amcl_jazzy/`：官方源码快照

该目录是从 ROS 2 Jazzy/Nav2 获取的 AMCL 阅读材料，不是自研 AMCL 的编译依赖：

| 路径 | 参考内容 |
|---|---|
| `include/nav2_amcl/` | 官方节点头文件、角度工具和可移植性辅助代码。 |
| `src/amcl_node.cpp`、`src/main.cpp` | 官方 ROS 节点生命周期、参数、通信和进程入口。 |
| `src/map/` | 官方地图、距离空间和射线相关的 C 实现。 |
| `src/motion_model/` | 官方差速和全向运动模型。 |
| `src/pf/` | 官方粒子滤波器、KD-tree、概率密度、向量和协方差工具。 |
| `src/sensors/laser/` | 官方激光传感器模型及其概率计算实现。 |
| `COLCON_IGNORE` | 阻止该源码快照被 colcon 当作工作区 ROS 包构建。 |
| `README.md`、`REFERENCE.md` | 记录快照来源、版本和阅读边界。 |

参考源码中的模块名称和 C/C++ 混合实现用于帮助理解算法，不应被 `mini_nav_core` 直接 include，也不应被 `mini_nav_nodes` 直接链接。自研实现需要保持自己的接口、测试和依赖方向。

#### 2.4.3 `docs/`：设计和协作接口

| 文件 | 作用 |
|---|---|
| `docs/architecture.md` | 记录包边界、文件职责、接口、依赖方向、数据流和启动方式；本次增加的 AMCL 设计主要在第 2 节和第 10 节。 |
| `docs/amcl_research.md` | 记录 AMCL 算法、官方实现和自研拆分的调研结论。 |
| `docs/amcl_code_walkthrough.md` | 按文件和调用路径导读官方 AMCL 源码，服务于实现对照。 |
| `docs/technicial-detail-elaboration.md` | 记录较细的技术推导和实现说明。 |
| `docs/project_status.md` | 记录里程碑、已验证功能、测试结果和当前限制。 |
| `docs/contributing.md` | 记录修改、构建、测试、验收和提交规范。 |

代码、话题、TF、参数或启动入口发生变化时，应先确认变化属于哪个包，再同步更新对应文档；尤其要保持第 2 节目录、第 5 节 ROS 接口契约和第 10 节 AMCL 细节之间的一致性。

## 3. 模块与依赖方向

```text
外部 ROS 2 / Gazebo / Nav2
          │
          ▼
mini_nav_bringup  ───────►  mini_nav_nodes  ───────►  mini_nav_core
        │                         │                         │
        └── 启动与资源编排         └── ROS 适配              └── 算法实现
```

依赖方向必须保持从上到下：

- `mini_nav_core` 不依赖 ROS 2 消息、节点或 TF；它处理地图、栅格坐标、路径规划和粒子定位。
- `mini_nav_nodes` 是 `mini_nav_core` 的 ROS 适配器，负责订阅地图、接收 RViz 交互消息、转换坐标并发布结果。
- `mini_nav_bringup` 负责组合 Gazebo、官方 Nav2 定位模块和自研节点，不把业务算法写进 launch 文件。
- `nav2_learning`、`nav2_stvl_demo` 和 `navigation2_tutorials` 是参考代码，不是 `mini_nav` 的运行依赖；生产代码不得引用它们的私有文件。

这里的设计目标是让核心模块成为有较深实现、较小接口的模块：调用者只需要提供 `Costmap2D`、起点和终点，A* 的搜索、障碍判断和路径回溯都隐藏在实现内部。ROS 话题、生命周期和消息转换集中在节点模块的接缝处；粒子滤波内部的运动模型、激光模型和空间分箱保持在核心模块，以保持修改的局部性。

## 4. 运行时数据流

完整链路有两个可互换入口：官方对照使用 `official_localization_astar.launch.py`，自研定位使用 `mini_localization_astar.launch.py`；下面的数据流的区别仅在 AMCL 实现。

```text
                                  ┌────────────────────┐
/map ─────────────────────────────►│ 官方或自研 AMCL    │──── map -> odom
                                  └──────┬─────────────┘
                                         ▲
                       /scan + odom→base TF
                                         │
                         ┌───────────────┴───────────────┐
                         │ Gazebo Waffle + ROS bridge   │
                         └─────────────────────────────┘

/map ─────────────────────────────►┌────────────────────┐
                                   │ CostmapPublisher  │
/initialpose ──清除旧路径──────────►│ Node               │◄── map -> base_footprint TF
/goal_pose ───────────────────────►└─────────┬──────────┘
                                             │ 膨胀规划图
                                             ▼
                                      ┌───────────────┐
                                      │ AStarPlanner  │
                                      └──────┬────────┘
                                             │ /mini_nav/global_path
                                             ▼
                                      ┌───────────────┐
                                      │ RViz          │
                                      │ SetInitialPose│
                                      │ SetGoal       │
                                      └───────────────┘

/initialpose ───────────────────────────────► AMCL
```


`/initialpose` 为 AMCL 提供定位初始猜测；`CostmapPublisherNode` 收到它时清除旧路径，并等待新的 `map -> odom` TF。每次收到 `/goal_pose`，A* 节点查询当时的 `map -> base_footprint` TF 作为起点。TF 缺失、过期或起终点落在规划安全区时不发布非空路径。路径跟踪模块仍需独立实现。

不启动仿真和定位的 `start_astar_navigation.sh` 显式设置 `planning.use_initial_pose_as_start:=true`，保留 RViz 手选起终点的独立算法演示；两套定位入口使用默认的 TF 起点。

## 5. ROS 接口契约

| 接口 | 发布者 | 订阅者 | 类型 | 语义 |
|---|---|---|---|---|
| `/map` | Nav2 map_server | 官方或自研 AMCL、`CostmapPublisherNode` | `nav_msgs/msg/OccupancyGrid` | 静态地图；可靠、Transient Local QoS |
| `/mini_nav/map` | `CostmapPublisherNode` | RViz | `nav_msgs/msg/OccupancyGrid` | 自研模块使用的地图可视化 |
| `/initialpose` | RViz | AMCL、`CostmapPublisherNode` | `geometry_msgs/msg/PoseWithCovarianceStamped` | AMCL 初始位姿；规划节点清除旧路径并等待新定位 TF |
| `/goal_pose` | RViz `SetGoal` | `CostmapPublisherNode` | `geometry_msgs/msg/PoseStamped` | A* 终点与终点朝向；规划起点取目标到来时的机器人 TF |
| `/mini_nav/global_path` | `CostmapPublisherNode` | RViz | `nav_msgs/msg/Path` | 自研 A* 输出的全局路径 |
| `/mini_nav/map_axes` | `CostmapPublisherNode` | RViz | `visualization_msgs/msg/MarkerArray` | 地图原点和坐标轴可视化 |
| `/scan` | Gazebo bridge | AMCL、RViz | `sensor_msgs/msg/LaserScan` | 激光观测 |
| `/odom` | Gazebo bridge | 当前无直接订阅；后续控制器使用 | `nav_msgs/msg/Odometry` | 局部里程计数据，并由仿真 TF 链路提供给 AMCL |
| `/tf` | Gazebo/AMCL 等 | TF listeners | `tf2_msgs/msg/TFMessage` | 动态坐标变换 |
| `/particle_cloud` | 当前入口的 AMCL | RViz | `nav2_msgs/msg/ParticleCloud` | AMCL 粒子群 |

### 坐标系约束

```text
map ──(官方或自研 AMCL 提供)──► odom ──(仿真里程计提供)──► base_footprint
                                                    │
                                  robot_state_publisher 提供静态 TF
                                                    ▼
                      base_link ─────────────────► base_scan
```

- `map` 是静态地图全局坐标系。
- `odom` 是连续但可能漂移的局部坐标系。
- `base_footprint` 是机器人平面基座坐标系。
- `base_link` 是机器人本体坐标系。
- `base_scan` 是激光雷达坐标系。
- 不允许发布静态 `map -> odom`。该变换必须由定位模块根据地图、激光和里程计估计产生。
- RViz 的 Fixed Frame 使用 `map`。

## 6. 核心模块接口

### `mini_nav_core::Costmap2D`

`Costmap2D` 的接口提供地图尺寸、分辨率、原点、栅格代价、栅格/世界坐标转换和基本地图绘制能力。它不负责 ROS 发布，也不读取 YAML 或 PGM 文件。

主要不变量：

- 坐标必须在地图范围内，越界输入不能破坏已有地图。
- 障碍和未知区域必须使用统一的代价值判断。
- 地图元数据改变时，栅格数据必须按新尺寸原子式重建。

### `mini_nav_core::AStarPlanner`

```text
Plan(const Costmap2D& costmap,
     MapLocation start,
     MapLocation goal) -> vector<MapLocation>
```

当前实现：

- 使用四邻域搜索。
- 使用曼哈顿距离作为启发式函数。
- 起点、终点越界或位于不可通行栅格时返回空路径。
- 没有路径时返回空路径。
- 返回的路径按起点到终点顺序排列。

### `CostmapPublisherNode`

该节点是 ROS 适配器，负责：

1. 从 `/map` 或 `map_file` 得到 `OccupancyGrid`。
2. 将占用栅格转换为 `Costmap2D`。
3. 将世界坐标转换为 A* 所需的栅格坐标。
4. 调用 `AStarPlanner`。
5. 将栅格路径转换为 `nav_msgs/msg/Path`。

重要参数：

- `map_topic`：外部地图话题，默认 `/map`。
- `map_file`：标准 trinary YAML/PGM 地图路径；为空时订阅 `map_topic`。
- `frame_id`：地图坐标系，默认 `map`。

## 7. 启动入口

执行前先加载统一环境：

```bash
cd /home/a/ros2_ws
source src/mini_nav/scripts/env_mini_nav.sh
```

| 用途 | 命令 | 内容 |
|---|---|---|
| 官方 AMCL 对照 | `ros2 launch mini_nav_bringup official_localization_astar.launch.py` | Waffle、map_server、官方 AMCL、自研 A*、RViz |
| 自研 AMCL 链路 | `ros2 launch mini_nav_bringup mini_localization_astar.launch.py` | Waffle、map_server、自研 AMCL、自研 A*、RViz |
| 静态 A* 演示 | `./src/mini_nav/scripts/start_astar_navigation.sh` | 直接加载地图文件、自研 A*、RViz；不启动仿真和定位 |
| 仿真接口 | `ros2 launch mini_nav_bringup waffle_sim.launch.py` | Waffle、Gazebo、传感器和里程计 |
| RViz 检查 | `ros2 launch mini_nav_bringup sim_check.launch.py` | 只启动 RViz，不伪造定位 TF |

完整入口的地图和参数通过 package-share 查找，默认使用 `mini_nav_bringup/maps/turtlebot3_map.yaml`。如果要测试工作区根目录的另一份地图，应通过 `map:=...` 显式传入。

## 8. 当前状态与后续接缝

已经完成的接缝：

- 地图服务到自研代价地图的接缝：`/map` → `CostmapPublisherNode`。
- 世界坐标到栅格坐标的接缝：`Costmap2D` 的转换接口。
- 栅格地图到路径消息的接缝：`AStarPlanner` → `/mini_nav/global_path`。
- 地图定位到机器人坐标的接缝：AMCL 提供 `map -> odom`。
- 定位计算与 TF 刷新的接缝：运动阈值控制粒子滤波成本，激光时间戳驱动缓存 `map -> odom` 重发。

下一阶段要增加的模块应放在 `/mini_nav/global_path` 和 `/cmd_vel` 之间：

```text
/mini_nav/global_path
          │
          ▼
PathFollower 适配器 ──► TF/AMCL 当前位姿 ──► TwistStamped /cmd_vel
```

这个 `PathFollower` 应拥有小而明确的接口，内部处理路径索引、朝向误差、速度限制、目标到达、超时和零速度保护。它不应被塞进 A* 或 `CostmapPublisherNode`，否则会让规划模块的接口变浅、测试范围变宽。

## 9. 验证原则

修改一个模块后，优先在它自己的接口上验证：

- `mini_nav_core`：用 `ament_cmake_gtest` 覆盖地图边界、障碍、无路径和坐标转换。
- `mini_nav_nodes`：验证地图消息转换、地图文件加载、起点终点边界、路径发布，以及 AMCL TF 缓存的初始状态、重时间戳和失效行为。
- `mini_nav_bringup`：运行 `ros2 launch ... --show-args`，检查 package-share 路径、参数和节点组合。
- 仿真联调：确认 `/clock`、`/scan`、`/odom`、`/tf`、`/map`、`/amcl_pose` 和 `map -> odom`，再验证 A* 路径。

不要用 RViz 是否“看起来有东西”替代话题、TF 和测试验证。

## 10. 自研 AMCL 架构（M1-M6 已实现，M7 待对照）

自研定位采用“独立算法核心 + ROS 适配节点 + 独立启动入口”的结构。官方 Nav2 AMCL 保留为稳定对照组，不直接覆盖当前已经验证过的 `official_localization_astar.launch.py`。

官方 Jazzy `nav2_amcl` 源码参考快照位于 `src/mini_nav/reference/nav2_amcl_jazzy/`。该目录保留了包结构、源码头部的版权/许可声明和上游版本说明，但带有 `COLCON_IGNORE`，只用于学习，不参与本项目构建或运行。

建议先阅读 `amcl_node.hpp/.cpp` 了解 ROS 适配层，再阅读 `pf/`、`sensors/laser/`、`motion_model/` 和 `map/`，将官方实现与下面规划的 ROS 无关核心逐项对应。

### 10.1 当前模块和文件

```text
mini_nav_core/localization/
├── types.hpp                         # Pose2D、Covariance3、Particle、PoseEstimate
├── laser_scan_data.hpp               # ROS 无关的激光扫描数据
├── localization_map.hpp/.cpp         # 地图快照、距离场和射线投射
├── motion_model.hpp                  # 运动模型抽象
├── differential_motion_model.hpp/.cpp
├── laser_model.hpp                   # 激光模型抽象
├── likelihood_field_model.hpp/.cpp
├── beam_model.hpp/.cpp
├── kd_tree.hpp/.cpp                  # ParticleFilter 内部的空间分箱/KLD 统计
└── particle_filter.hpp/.cpp          # 初始化、预测、权重、重采样和估计

mini_nav_nodes/
├── include/mini_nav_nodes/amcl_node.hpp
├── src/amcl_node.cpp                 # AmclNode 生命周期、ROS 适配和 TF 缓存
└── test/test_amcl_tf_cache.cpp       # TF 缓存、重时间戳和失效测试
```

定位算法直接放入已有的 `mini_nav_core` ROS 2 CMake 包中，作为该包内的 `localization/` 子模块。这样不新增包，也不改变现有 `Costmap2D` 和 `AStarPlanner` 的接口。定位算法本身仍然不依赖 `rclcpp`、ROS 消息或 TF，只复用同包的 `Costmap2D`；它不依赖 `AStarPlanner`。

### 10.2 依赖方向

```text
mini_nav_bringup
        │
        └── mini_nav_nodes ─────────► mini_nav_core
                 │                       │
                 │                       ├── Costmap2D + A*
                 │                       └── localization/*
                 └── ROS 适配 amcl_node
```

具体职责如下：

- `mini_nav_core` 负责 ROS 无关的地图、A* 和定位算法，同时保持既有 `Costmap2D` 与 `AStarPlanner` 接口兼容。
- `mini_nav_core/localization/*` 负责概率定位算法，包括运动模型、激光模型和粒子滤波。
- `mini_nav_nodes` 中的 `amcl_node` 负责 ROS 消息转换、QoS、TF 查询、生命周期和发布。
- `mini_nav_bringup` 负责把 map_server、Waffle 仿真、自研定位节点、A* 节点和 RViz 组合起来。
- 官方 `nav2_amcl` 只在官方对照入口中运行；自研入口不能同时启动两个 `map -> odom` 发布者。

这里不提前抽象出虚拟的 `MapInterface`。当前只有 `Costmap2D` 一个地图适配器，定位子模块直接使用它能保持接口更小；等第二种地图实现真正出现时，再引入新的接缝。

### 10.3 算法核心的最小接口

核心模块的公共接口保持小而深，`AmclNode` 不需要知道内部的粒子组织方式：

```text
ParticleFilter(options)
    Initialize(const Pose2D& pose, const Covariance3& covariance)
    InitializeGlobal(const LocalizationMap& map)
    MotionUpdate(previous_odom_pose, current_odom_pose)
    SensorUpdate(scan, map, base_to_laser_pose)
    NormalizeWeights() / Resample()
    Estimate() -> PoseEstimate
```

模块边界：

- `LocalizationMap` 负责地图快照、自由栅格/障碍判断、障碍距离查询、自由栅格采样和射线投射。
- `DifferentialMotionModel` 负责差速里程计增量到粒子位姿的带噪声预测。
- `LaserModel` 的具体实现负责根据地图和扫描计算粒子权重。
- `ParticleFilter` 负责调用上述模块，组织初始化、运动更新、传感器更新、归一化、重采样和估计。
- `PoseBinIndex` 的公共接口只服务 `ParticleFilter`，外部调用者不能依赖粒子分箱细节。

当前核心输入使用弧度和米，支持 yaw 为 0 的二维静态地图；核心不依赖 ROS 消息、节点或 TF。

### 10.4 ROS 适配节点的职责

`amcl_node` 已实现为生命周期节点，以便在地图、TF、参数和发布器准备好之后再处理传感器数据。

`on_configure()` 创建一个互斥回调组，并将其设置为自动加入节点 executor。地图、初始位姿和激光订阅都绑定到这个回调组；`main.cpp` 使用普通 `rclcpp::spin()` 驱动节点，因此不需要额外手动添加回调组。这个配置是运行时约束：如果回调组关闭自动加入 executor，ROS 图仍会显示订阅连接，但地图、初始位姿和激光回调不会被调度，最终不会产生定位输出。

输入：

| 输入 | 类型 | 节点职责 |
|---|---|---|
| `map_topic`，默认 `/map` | `nav_msgs/msg/OccupancyGrid` | 转换为独立的 `Costmap2D` 快照 |
| `scan_topic`，默认 `/scan` | `sensor_msgs/msg/LaserScan` | 在扫描时间戳查询传感器 TF，转换为 core 观测 |
| `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 校验 frame、协方差和地图范围后初始化粒子 |
| TF | `odom -> base_footprint`、`base_footprint -> base_scan` | 计算里程计增量和激光安装位姿 |

输出：

| 输出 | 类型 | 初始规划 |
|---|---|---|
| `pose_topic` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 当前实现发布 `/amcl_pose`，与官方入口保持可对照 |
| `particle_cloud_topic` | `nav2_msgs/msg/ParticleCloud` | 供 RViz 观察收敛，后续可改为项目自有可视化消息 |
| `tf_broadcast` 控制的 `map -> odom` | TF | 自研独立入口中开启；滤波更新时重算并缓存，未达运动阈值时按激光时间戳重发；对照实验中严格单发布者 |

节点层只做适配，不实现粒子权重或射线投射。TF 缓存是 ROS 输出语义的一部分，因此保留在 `AmclNode`，不下沉到 ROS 无关的 `ParticleFilter`。节点还需要显式处理 QoS：地图使用可靠、Transient Local；激光按传感器数据 QoS；所有仿真节点统一使用 `use_sim_time`。

### 10.5 启动入口隔离

现有入口保持不变：

```text
official_localization_astar.launch.py
    Waffle + map_server + 官方 AMCL + A* + RViz
```

新增入口单独验证自研定位：

```text
mini_localization_astar.launch.py
    Waffle + map_server + mini_nav_amcl_node + A* + RViz
```

自研入口不启动 `nav2_amcl`，但仍可以复用 map_server 和仿真传感器。这样官方 AMCL 是对照实现，自研 AMCL 是被测实现，二者可以使用相同地图、起点和仿真轨迹分别运行。

### 10.6 实现顺序和验收点

1. **M1：粒子滤波核心**（已完成）。验证初始化、权重归一化、重采样、均值、角度均值和协方差。
2. **M2：运动模型**（已完成）。给定固定 odometry 增量，验证粒子平移/旋转方向和噪声随参数变化。
3. **M3：射线投射与激光模型**（已完成）。在小地图上验证命中墙、越界和未知区域处理。
4. **M4：核心跟踪**（已接入）。给定初始位姿、运动和扫描序列，验证估计不会发散。
5. **M5：ROS 适配**（已完成基础接入）。验证 pose、particle cloud、时间戳、QoS 和 TF 输入。
6. **M6：TF 与独立启动**（已完成）。确认 `map -> odom -> base_footprint -> base_scan` 完整，并由自研 AMCL 发布 `map -> odom`；已在 Gazebo teleop 与 RViz 中验证缓存 TF 按激光时间重发后的连续显示。
7. **M7：官方对照**（待完成）。比较收敛时间、定位误差、粒子分布和丢失后的恢复能力。

核心算法已接入 `mini_nav_core` 单元测试；自研节点和独立入口继续单独验证，避免把算法问题与 DDS、生命周期和 TF 问题混在一起。

几个容易忽略的实现细节：

1. AMCL 的地图订阅是 reliable + transient local；地图发布者也应保留地图，启动顺序变化时才不会因为错过唯一一条地图消息而空等。[Jazzy 地图订阅 QoS](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1297-L1306)
2. AMCL 的激光输入使用 sensor-data QoS，并用 TF MessageFilter 等待目标时间的 TF；所以“`/scan` 有数据”不等于“AMCL 已经能够使用这些数据”。[Jazzy 激光 MessageFilter](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1424-L1442)
3. `map` 的 `header.frame_id` 应与 AMCL 的 `global_frame_id` 一致。Jazzy 源码会对此发出警告；当前项目两者都应为 `map`。[Jazzy 地图处理](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1311-L1346)
4. `base_frame_id` 必须与实际 TF 树一致。项目当前使用 `base_footprint`，而不是默认值也常见的 `base_link`；这个选择来自 Waffle 的实际 TF 配置。[项目 AMCL 配置](../mini_nav_bringup/config/amcl_waffle.yaml)
