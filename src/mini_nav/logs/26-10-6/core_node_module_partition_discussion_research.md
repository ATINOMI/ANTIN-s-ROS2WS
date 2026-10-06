# mini_nav core 与 node 层划分：讨论方案与源码调研

- 日期：2026-10-06
- 工作区：`/home/a/ros2_ws`
- 调研对象：`src/mini_nav/` 当前工作树
- 调研时 HEAD：`f6bcc5df2a8da0dc10a63a56c4beccdfeb5b1787`

> 状态：本文件记录讨论结论和当前源码调研结果，不是重构完成报告。调研时工作树已有未提交修改，因此 HEAD 不能完整代表本次读取的源码。此次仅新增本文档，没有移动源码，也没有重新构建、运行测试或启动仿真。

## 1. 目标与已确定的讨论结论

目标是让算法职责和节点运行职责更容易从目录中辨认，并解除目前不必要的跨模块依赖。两层采用不同的组织依据：

- **core 按算法职责组织**：导航基础类型、地图、定位、碰撞检查、规划和控制。
- **nodes 按节点与运行职责组织**：AMCL、FAST-LIO2、地图管理、导航任务管理、速度保护和路径跟踪管理。

讨论中确定的目录名称与归属如下：

| 讨论项 | 最终结论 |
|---|---|
| 通用基础数据目录 | 使用 `nav_types/`，保存共用点、位姿、变换和路径定义 |
| 共享几何安全检查 | 使用 `collision_checker/`，职责是碰撞、间距和连续扫掠检查 |
| 规划与控制 | 放在 core 的 `navigator/planner/`、`navigator/controller/` |
| 导航任务管理器 | 留在 nodes，使用 `navigation_task_manager/` |
| 速度保护器 | 留在 nodes，使用 `velocity_guard/` |
| 路径跟踪节点 | 使用 nodes 的 `tracker_manager/` |
| AMCL 与 FAST-LIO2 | nodes 中分别使用 `amcl/`、`fastlio2/` 专属目录 |
| 地图相关节点 | `costmap_publisher.cpp`、`local_costmap_node.cpp` 进入 `map_manager/` |
| 地图辅助头文件 | `collision_map.hpp`、`cloud_validation.hpp`、`costmap_display.hpp` 进入 nodes 的 `map_manager/` |
| nodes 的头文件 | 按上述分组组织到 `include/mini_nav_nodes/`；FAST-LIO2 的项目接入头文件也归入 `fastlio2/` |

本轮不要求把每个目录变成独立 ROS 包，也不要求把每个辅助文件变成独立节点。基础节点的可执行程序、ROS 接口和运行行为应在结构迁移阶段保持兼容。

## 2. 当前源码调研结果

### 2.1 基础 core 库的实际组成

[mini_nav_core/CMakeLists.txt](../../mini_nav_core/CMakeLists.txt) 第 10–22 行当前将以下实现编入同一个基础库：

- 地图：`costmap_2d.cpp`、`inflation_layer.cpp`、`rolling_obstacle_grid.cpp`。
- 导航：`astar_navigator.cpp`、`path_postprocessor.cpp`、`path_tracker.cpp`。
- AMCL：定位地图、差分运动模型、似然场模型、束模型、位姿分箱索引和粒子滤波。

FAST-LIO2 底层文件虽然位于 `mini_nav_core/src/localization/fastlio2/`，目前不参与这个基础库的编译。基础库也没有链接 ROS、PCL 或 FAST-LIO2。

### 2.2 core 当前存在的职责交叉

| 源码证据 | 当前情况 | 整理方向 |
|---|---|---|
| [path_postprocessor.hpp](../../mini_nav_core/include/mini_nav_core/navigator/path_postprocessor.hpp)，第 18、33、59 行 | 同时定义 `PathPoint`、`CollisionGeometry` 和连续圆形扫掠接口 | 路径点归 `nav_types`；碰撞几何与扫掠归 `collision_checker`；后处理保留在 planner |
| [rolling_obstacle_grid.hpp](../../mini_nav_core/include/mini_nav_core/map/rolling_obstacle_grid.hpp)，第 13 行 | 地图模块包含路径后处理头文件以取得共用类型 | 改为依赖基础类型，消除 map 对路径后处理的依赖 |
| [astar_navigator.hpp](../../mini_nav_core/include/mini_nav_core/navigator/astar_navigator.hpp)，第 13 行 | A* 为使用碰撞几何而依赖路径后处理头文件 | A* 与后处理共同依赖碰撞检查器 |
| [path_tracker.hpp](../../mini_nav_core/include/mini_nav_core/navigator/path_tracker.hpp)，第 11–12 行 | 控制器依赖定位的 `types.hpp` 和路径后处理头文件 | 共用位姿归 `nav_types`，共用碰撞接口归 `collision_checker` |

因此，第一步不仅是把 `navigator/` 分成两个子目录，还要拆出被多个模块使用的数据和碰撞职责。

### 2.3 nodes 当前包含六个基础 C++ 节点

依据 [mini_nav_nodes/CMakeLists.txt](../../mini_nav_nodes/CMakeLists.txt) 和 [main.cpp](../../mini_nav_nodes/src/main.cpp)，六个节点复用入口文件，通过编译宏构建成六个独立可执行程序。

| 可执行程序 | 当前源文件 | 主要职责 | 是否链接基础 core 库 |
|---|---|---|---|
| `mini_nav_amcl_node` | `src/amcl_node.cpp` | 地图、激光、初值接入；生命周期；AMCL 调用；位姿、粒子、质量与 TF 发布 | 是 |
| `costmap_publisher_node` | `src/costmap_publisher.cpp` | 地图准备、代价图、动态观测、规划 Action、A* 与后处理调用、路径和地图发布 | 是 |
| `local_costmap_node` | `src/local_costmap_node.cpp` | 激光及可选点云接入、观测时间 TF、滚动障碍图、碰撞快照和有效性发布 | 是 |
| `path_follower_node` | `src/path_follower_node.cpp` | FollowPath、输入新鲜度检查、跟踪器调用、候选速度与执行反馈 | 是 |
| `navigation_manager_node` | `src/navigation_manager_node.cpp` | NavigateToPose、子 Action、任务身份、取消、抢占、重规划、期限和运动许可 | 否 |
| `velocity_guard_node` | `src/velocity_guard_node.cpp` | 独立看门狗、命令与任务心跳检查、速度转发和失效停车 | 否 |

任务管理器和速度保护器目前就位于 nodes。本方案不把它们迁入 core。

### 2.4 地图辅助头文件的实际职责

| 文件 | 当前职责 | 目标位置 |
|---|---|---|
| [collision_map.hpp](../../mini_nav_nodes/include/mini_nav_nodes/collision_map.hpp) | `DecodeCollisionMap` 校验 ROS 碰撞快照，解码原始栅格和连续障碍点 | `include/mini_nav_nodes/map_manager/` |
| [cloud_validation.hpp](../../mini_nav_nodes/include/mini_nav_nodes/cloud_validation.hpp) | `ValidCollisionCloud` 校验 PointCloud2 的布局、字段和数据大小 | 同上 |
| [costmap_display.hpp](../../mini_nav_nodes/include/mini_nav_nodes/costmap_display.hpp) | `CostToOccupancyValue` 将内部代价值转换为 ROS/RViz 占据显示值 | 同上 |

这些文件处理地图相关的 ROS 数据语义。它们不等同于 core 的车体碰撞算法。路径跟踪节点可以引用 `map_manager/collision_map.hpp`，目录归属不表示只有地图节点可以使用它。

`CollisionMap.msg` 仍放在 `mini_nav_nodes/msg/`，继续由该包生成消息类型。

### 2.5 FAST-LIO2 目前只完成了部分分离

当前目录与构建说明见 [localization_backends.md](../../docs/localization_backends.md)、[计算内核说明](../../mini_nav_core/src/localization/fastlio2/README.md)和 [ROS 接入说明](../../mini_nav_nodes/src/localization/fastlio2/README.md)。

| 部分 | 当前位置 | 调研结论 |
|---|---|---|
| IKFoM、过程模型、ikd-Tree、旋转数学 | `mini_nav_core/src/localization/fastlio2/` | 已按源码位置分出；计算内核不引用 ROS 消息，仍有 Eigen/PCL 依赖 |
| 主流程、IMU、点云处理 | `mini_nav_nodes/src/localization/fastlio2/fast_lio/` | 算法与 ROS 接入仍混合 |
| 初值邻域 ICP | 同级 `icp_relocalization/` | 独立 ROS 包；本次没有完成它内部全部职责的逐函数拆分调研 |
| mini_nav 接入 | 同级 `adapter/` | 输入适配、坐标转换、质量、TF、人工初值、会话和后端进程管理；另有建图操作代码 |

混合情况有明确的源码依据：

- [laserMapping.cpp](../../mini_nav_nodes/src/localization/fastlio2/fast_lio/src/laserMapping.cpp)：包含 `h_share_model`、`LaserMappingNode`、传感器回调、滤波更新和 ROS 输出。第 172 行是点面观测相关函数，第 294 行定义 ROS 节点，第 1201 行调用迭代滤波更新。
- [IMU_Processing.hpp](../../mini_nav_nodes/src/localization/fastlio2/fast_lio/src/IMU_Processing.hpp)：执行初始化、预测和点云去畸变，同时直接保存 ROS IMU 消息，并用 `rclcpp::Time` 读取时间。
- [preprocess.h](../../mini_nav_nodes/src/localization/fastlio2/fast_lio/src/preprocess.h)：第 144–145 行的处理接口直接接收 Livox 或 PointCloud2 消息。
- [common_lib.h](../../mini_nav_nodes/src/localization/fastlio2/fast_lio/include/common_lib.h)：混合算法类型与 ROS 类型；`MeasureGroup` 内部保存 `sensor_msgs::msg::Imu` 指针，`Pose6D` 使用 ROS 生成消息。

[fast_lio/CMakeLists.txt](../../mini_nav_nodes/src/localization/fastlio2/fast_lio/CMakeLists.txt) 第 101–108 行通过相对路径定位 core 中的 FAST-LIO2 文件，再把 `ikd_Tree.cpp` 直接编入 `fastlio_mapping`。目前不存在供这个节点单独链接的完整 FAST-LIO2 算法库接口。

因此，不能把当前情况描述为“完整算法已在 core，nodes 只负责 ROS”。但这些三维依赖也没有进入基础二维导航库。

## 3. core 层目标结构

### 3.1 模块职责

| 模块 | 职责 | 不应承担的内容 |
|---|---|---|
| `nav_types` | 共用点、位姿、变换、路径定义与紧邻类型的基础操作 | ROS 消息、粒子细节、FAST-LIO2 三维状态 |
| `map` | 栅格、坐标转换、观测维护、滚动、过期和膨胀 | ROS 订阅发布、规划任务管理 |
| `localization/amcl` | 自研粒子滤波和定位模型 | ROS 生命周期、TF 广播 |
| `localization/fastlio2` | FAST-LIO2 数据、预处理、IMU 预测、去畸变、点面匹配、滤波和地图维护 | ROS 消息解析、发布、后端子进程管理 |
| `collision_checker` | 原始障碍与车体的间距、未知区域、边界和连续扫掠检查 | 路径搜索、控制候选生成、ROS 碰撞消息解码 |
| `navigator/planner` | A*、安全终点选择、简化、加密、平滑和最终路径校验 | 发布速度、发送 ROS Action |
| `navigator/controller` | 跟踪、限速限加速度、运动与制动预测、候选检查、进展判断 | TF 查询、消息新鲜度判断、整项导航任务编排 |

### 3.2 目录结构

以下是目标结构。FAST-LIO2 新接口名称属于拟定名称，具体函数、数据所有权和异常契约需要在实现前细化；`...` 表示保留的同类文件，不表示删除现有文件。

```text
mini_nav_core/
├── include/mini_nav_core/
│   ├── nav_types/
│   │   ├── point_2d.hpp
│   │   ├── pose_2d.hpp
│   │   ├── transform_2d.hpp
│   │   └── path.hpp
│   ├── map/
│   │   ├── costmap_2d.hpp
│   │   ├── inflation_layer.hpp
│   │   └── rolling_obstacle_grid.hpp
│   ├── localization/
│   │   ├── amcl/
│   │   │   ├── particle_filter.hpp
│   │   │   ├── localization_map.hpp
│   │   │   ├── motion_model.hpp
│   │   │   ├── laser_model.hpp
│   │   │   └── ...                    # AMCL 专属模型、参数和类型
│   │   └── fastlio2/
│   │       ├── fastlio_estimator.hpp   # 拟新增的定位计算接口
│   │       └── fastlio_types.hpp       # 算法输入、状态与结果
│   ├── collision_checker/
│   │   ├── collision_geometry.hpp
│   │   └── collision_checker.hpp
│   └── navigator/
│       ├── planner/
│       │   ├── astar_navigator.hpp
│       │   └── path_postprocessor.hpp
│       └── controller/
│           └── path_tracker.hpp
├── src/
│   ├── map/
│   │   ├── costmap_2d.cpp
│   │   ├── inflation_layer.cpp
│   │   └── rolling_obstacle_grid.cpp
│   ├── localization/
│   │   ├── amcl/
│   │   │   └── ...                    # 保留现有 AMCL 实现
│   │   └── fastlio2/
│   │       ├── fastlio_estimator.cpp
│   │       ├── imu_processing.*       # IMU 初始化、预测、去畸变
│   │       ├── pointcloud_preprocess.*
│   │       ├── ...                    # 匹配、地图维护及内部类型
│   │       ├── IKFoM_toolkit/
│   │       ├── ikd-Tree/
│   │       ├── use-ikfom.hpp
│   │       ├── so3_math.h
│   │       └── Exp_mat.h
│   ├── collision_checker/
│   │   └── collision_checker.cpp
│   └── navigator/
│       ├── planner/
│       │   ├── astar_navigator.cpp
│       │   └── path_postprocessor.cpp
│       └── controller/
│           └── path_tracker.cpp
├── test/
├── CMakeLists.txt
└── package.xml
```

### 3.3 依赖规则

- map 不依赖 planner 或 controller。
- planner 与 controller 共同使用地图、导航类型和碰撞检查器，二者不互相包含实现头文件。
- controller 不依赖 AMCL 的粒子类型；它使用通用位姿和路径。
- 定位算法不依赖导航任务管理器或速度保护器。
- FAST-LIO2 的三维数据留在其专属类型中，不把 PCL/Eigen 三维类型塞进公共二维 `nav_types`。
- 节点查询 TF，再向算法传入明确方向的变换；时间由调用方传入，并区分观测时间、仿真时间和单调时间。
- 输入快照的坐标系、有效时间、所有权和引用生命周期必须在接口中说明，避免把目录重排变成数据语义变化。

## 4. nodes 层目标结构

### 4.1 源文件

```text
mini_nav_nodes/
├── src/
│   ├── amcl/
│   │   └── amcl_node.cpp
│   ├── fastlio2/
│   │   ├── fast_lio/                  # 保留独立 ROS 包
│   │   │   ├── src/
│   │   │   │   ├── main.cpp
│   │   │   │   └── node/
│   │   │   │       ├── fastlio_node.cpp
│   │   │   │       ├── sensor_input.cpp
│   │   │   │       ├── ros_output.cpp
│   │   │   │       └── node_parameters.cpp
│   │   │   ├── msg/
│   │   │   ├── config/
│   │   │   ├── launch/
│   │   │   ├── rviz_cfg/
│   │   │   ├── CMakeLists.txt
│   │   │   └── package.xml
│   │   ├── icp_relocalization/        # 保留独立 ICP ROS 包
│   │   └── adapter/
│   │       ├── adapter.py
│   │       ├── mini_nav_localizer.py
│   │       ├── localization_backend.py
│   │       ├── localization_geometry.py
│   │       └── mapping_controls.py
│   ├── map_manager/
│   │   ├── costmap_publisher.cpp
│   │   └── local_costmap_node.cpp
│   ├── navigation_task_manager/
│   │   └── navigation_manager_node.cpp
│   ├── velocity_guard/
│   │   └── velocity_guard_node.cpp
│   ├── tracker_manager/
│   │   └── path_follower_node.cpp
│   └── main.cpp                       # 六个基础节点的共享入口
├── include/mini_nav_nodes/
│   ├── amcl/
│   │   └── amcl_node.hpp
│   ├── fastlio2/
│   │   ├── fastlio_node.hpp
│   │   ├── sensor_input.hpp
│   │   ├── ros_output.hpp
│   │   └── node_parameters.hpp
│   ├── map_manager/
│   │   ├── costmap_publisher.hpp
│   │   ├── local_costmap_node.hpp
│   │   ├── collision_map.hpp
│   │   ├── cloud_validation.hpp
│   │   └── costmap_display.hpp
│   ├── navigation_task_manager/
│   │   └── navigation_manager_node.hpp
│   ├── velocity_guard/
│   │   └── velocity_guard_node.hpp
│   └── tracker_manager/
│       └── path_follower_node.hpp
├── msg/
│   └── CollisionMap.msg
├── test/
├── CMakeLists.txt
└── package.xml
```

FAST-LIO2 的 `fastlio_node` 等文件是拆分后的目标，不是当前已有文件。独立包自己的第三方辅助头文件和资源仍由该包管理，不全部搬入项目公开头文件目录。

### 4.2 各组职责

| 分组 | 运行职责 |
|---|---|
| `amcl` | ROS 消息、生命周期、初始位姿、定位质量、粒子可视化和 TF 广播 |
| `fastlio2` | 后端 ROS 包、输入输出、参数，以及与 mini_nav 的质量和会话接入 |
| `map_manager` | 地图与观测接入、校验、消息转换、地图发布和现有规划服务接入 |
| `navigation_task_manager` | 任务状态、Action 编排、取消与抢占、重规划、期限和运动许可 |
| `velocity_guard` | 独立时间与心跳检查、速度校验、转发或零速输出 |
| `tracker_manager` | FollowPath、TF 查询、数据新鲜度和有效性检查、控制算法调用、候选速度与反馈 |

### 4.3 地图管理与规划接入的关系

`costmap_publisher.cpp` 目前同时处理地图和规划，讨论确定将它放入 `map_manager/`。这是当前节点的目录归属，不等于它变成纯地图发布器。

实施时应让地图计算归 core/map，A* 与路径后处理归 core/navigator/planner，ROS 规划请求与结果转换留在现有节点。是否以后拆成独立规划节点，需要结合职责体量再决定，不作为此次目录整理的默认动作。

## 5. FAST-LIO2 专项拆分

### 5.1 ROS 接入职责

| 拟定文件 | 职责 |
|---|---|
| `fastlio_node.*` | 建立节点对象、持有算法实例、协调数据处理与输出、处理节点已有控制接口 |
| `sensor_input.*` | ROS IMU、PointCloud2、Livox 消息的格式校验和转换；维护接入所需缓存 |
| `ros_output.*` | 将算法结果转换为里程计、点云、轨迹、诊断和 TF 消息 |
| `node_parameters.*` | 声明、读取、校验 ROS 参数，并组装算法配置 |

这些文件仍构成同一个 FAST-LIO2 可执行程序。只有明显独立且重复的职责才抽取辅助类，避免把每个回调都包装成一层转发。

节点输入转换后，算法接收带时间的 IMU 样本、带点时间的点云和标定数据。IMU 初始化、预测、测量时间组织、去畸变、残差、滤波更新和地图维护属于算法。ROS 消息缓冲与算法时序不能仅按变量名机械迁移，需结合所有权和更新顺序设计。

### 5.2 现有混合头文件

| 当前文件 | 需要拆开的内容 |
|---|---|
| `IMU_Processing.hpp` | 纯数据和 IMU/去畸变计算进入 core；ROS 消息接入进入 sensor_input |
| `preprocess.h` / `preprocess.cpp` | 点云算法进入 core；ROS/Livox 布局解析进入 sensor_input |
| `common_lib.h` | 算法状态与测量结构进入 fastlio_types 或私有类型；ROS 消息别名和转换留在 nodes |
| `laserMapping.cpp` | 匹配、滤波、地图计算进入估计器实现；回调、参数、发布和运行协调进入 node 文件 |

第三方原始代码、注释与许可保留。拆分需要记录原文件与目标文件的对应关系；只有解除 ROS 依赖所必需的接口调整才进入算法改造。

### 5.3 node 与 adapter 的区别

`fast_lio` 中的 node 让算法作为 ROS 节点运行；`adapter` 把该后端接入 mini_nav 的导航约定。

人工初值、后端会话、定位 epoch 和导航失效处理沿用当前已有的管理归属。不要在新 node 中再建一套重复的会话状态机。`mapping_controls.py` 是建图操作配套，留在后端专属目录中，不能误当作定位算法。

ICP 包仍保留其独立包名与构建入口。ICP 内部算法是否提取、如何表达配准结果和质量，需要补充逐函数调研后再明确；本报告不宣称这一部分已设计或实现完成。

## 6. 构建与路径迁移要求

目录移动不能只修改 include：

1. 更新基础 core 与 nodes 的 CMake 源文件路径、头文件引用和测试引用。
2. 将 `include/costmap_publisher.hpp` 移至统一的 `include/mini_nav_nodes/map_manager/`。
3. 将 nodes 的 `src/localization/fastlio2/` 移至 `src/fastlio2/` 后，更新独立包入口、构建脚本、Python 安装路径、源码清单和文档引用。
4. FAST-LIO2 专属 ROS 头文件集中后，独立包必须显式声明头文件搜索路径和相关构建依赖；不能依赖未声明的偶然路径。
5. 为 FAST-LIO2 计算建立独立、可导出的库目标；独立构建入口启用并安装它，fast_lio 消费其导出目标，不直接编译 core 私有源码。
6. 基础二维构建不因这个独立库默认引入 PCL、FAST-LIO2 或三维 ROS 消息依赖。具体 CMake 选项、目标名称及导出方式在实施阶段确定。
7. 保持第三方包的发现隔离。移动嵌套 ROS 包时核对 COLCON_IGNORE 与显式 --base-paths，避免主工作区自动发现范围改变。

当前需要关注的构建入口包括 [build_localization.sh](../../mini_nav_fastlio/scripts/build_localization.sh)、[build_fastlio.sh](../../mini_nav_fastlio/scripts/build_fastlio.sh) 和 [scurm_sim/CMakeLists.txt](../../mini_nav_fastlio/scurm_sim/CMakeLists.txt)。当前脚本明确引用旧的 `src/localization/fastlio2/` 路径；Python 接入由 scurm_sim 安装。

## 7. 实施顺序与验收计划

### 阶段一：目录迁移

按已确定名称移动源文件和头文件，更新构建、引用、脚本与文档。保留现有文件名、类名和 ROS 接口；保护现有注释、备份与无关修改。不同时改变算法参数或导航行为。

### 阶段二：二维 core 职责拆分

抽出 nav_types 和 collision_checker；把规划与控制移入 navigator 的对应子目录。解除 map 对路径后处理、controller 对定位专属类型的依赖。复用现有针对坐标边界、未知区域、扫掠、真实起点、终点容差、路径后处理和制动安全的测试。

### 阶段三：FAST-LIO2 算法与 ROS 接入分离

先明确输入、状态、输出、时间和重置接口，再拆 IMU、预处理、匹配与地图计算，建立独立计算库，最后整理 node 文件。保留现有原始版证据用于对照；不能只凭编译成功认定定位结果等价。

### 阶段四：运行验收

- 按 Jazzy 工作区约定先构建，再 source overlay，运行所有改动包的相关测试。
- 修改 launch 或后端入口时先检查 `--show-args`。
- 验证基础 AMCL 单目标导航、取消、新目标抢占、停车后重规划和输入失效停车。
- FAST-LIO2 单独验证传感器转换、时间组织、初始化、定位、重新设置初值、会话失效停车及关闭清理。
- 用隔离 ROS_DOMAIN_ID 和 GZ_PARTITION 运行仿真；完整导航入口保持唯一动态 map→odom 发布者和唯一 /cmd_vel 发布者。
- 对比重构前后点时间、去畸变结果、位姿、匹配质量和地图结果。未执行的检查必须明确记录为未验证。

本次文档编写没有执行以上构建、测试或运行验收。测试计划不代表测试结果。

## 8. 本次调研的范围与保留事项

本报告使用当前本地源码、构建文件和项目文档作为证据，记录用户讨论中确定的命名和归属，没有重新联网研究上游实现。

已经确定：两层组织原则、主要目录名称、基础节点归属、地图辅助头文件归属，以及 FAST-LIO2 需要解除算法与 ROS 耦合的方向。

实施前还需要细化：FAST-LIO2 估计器的函数接口、输入与快照所有权、时间语义、库目标与导出方式、ICP 内部拆分，以及迁移过程中是否需要短期兼容头文件。

FAST-LIVO2、RViz 插件、bringup、仿真资源和其他工作区包不在本次目录重构方案的直接范围内。相关路径如果受到移动影响，应修正引用，但不能把此方案扩展成对所有后端的重新实现。
