# core 与 node 模块重构实施记录

日期：2026-10-06。重构前 Git 留档：`f2218c9a8109ffcf4c06da17615ff582b9cace6d`，提交名 `Save mini_nav before core and node module refactor`。此提交包含当时 mini_nav 源码、测试、文档与 [划分讨论](core_node_module_partition_discussion_research.md)，排除生成目录、运行日志和无关工程改动。

## 已实现的划分

最终完整目录见 [架构说明](../../docs/architecture.md)，定位计算与接入文件清单见 [定位后端说明](../../docs/localization_backends.md)。

| core 算法目录 | 内容 |
|---|---|
| `nav_types/` | 共用点、路径、位姿、角度和刚体复合；保留原 PathPoint / AMCL Pose2D 类型别名 |
| `map/` | Costmap2D、InflationLayer、RollingObstacleGrid；不再包含路径后处理头文件 |
| `collision_checker/` | CollisionGeometry、冲突信息、连续圆盘扫掠和共享距离计算 |
| `navigator/planner/` | A*、安全路径后处理；依赖共用碰撞接口 |
| `navigator/controller/` | PathTracker；共用位姿移出 AMCL，碰撞检查移出后处理 |
| `localization/amcl/` | 自研粒子滤波、地图和观测/运动模型，公开头文件同步归组 |
| `localization/fastlio2/` | FAST-LIO2 估计器、IMU、预处理、IKFoM 与 ikd-Tree；可选独立计算库 |

nodes 按运行职责划分为 `amcl/`、`fastlio2/`、`map_manager/`、`navigation_task_manager/`、`velocity_guard/`、`tracker_manager/`，公开头文件对应归组。地图管理目录包括两个地图节点及 `collision_map.hpp`、`cloud_validation.hpp`、`costmap_display.hpp`。六个基础节点继续复用根 `src/main.cpp`，导航任务管理器和速度保护器仍不链接基础 core 库。

FAST-LIO2 独立包的 `src/main.cpp` 只负责进程入口；`src/node/` 分为节点调度、参数、传感器输入及 ROS 输出。项目头文件集中到 nodes 的 `fastlio2/`，不为每个 cpp 建立空转发头文件。adapter 与 ICP 独立包保留原接口，Python 安装和测试路径同步更新。

## FAST-LIO2 的实际计算边界

默认 `MINI_NAV_BUILD_FASTLIO2=OFF`，二维核心和 AMCL 不链接 PCL。独立构建传入 ON，额外导出 `mini_nav_core::mini_nav_fastlio2_core`，依赖 Eigen/PCL/OpenMP，无 ROS 消息、rclcpp 或 TF 依赖。fast_lio 链接该 target，通过公开输入输出接口调用，不再引用 core 私有计算文件。

`FastlioEstimator` 保存实例状态、点面观测、滤波与地图；IMU 使用普通时间和向量，点云预处理接受已解码 PCL/Livox 数据。节点保留 ROS 队列、同步、参数、消息转换和发布。借用结果点云在下一帧更新，调用者串行消费；首次先验初始化与运行中重置的生命周期在公开接口和 docs 中注明。

保留原算法注释与许可。IKFoM 动态观测回调改为实例绑定，避免多个估计器共用全局回调状态。提取时发现并修复有效点云 clear 后下标写入的问题，先按输入容量 resize，再按有效点数收缩；缺失日志目录时不解引用空 FILE 指针。日志路径来自配置，不再依赖 nodes 源码目录。CSV 中预处理耗时属于输入层，本轮计算库该列写 0；不将它当作实测预处理耗时。数学滤波公式未改写。

## 验证结果

重构前，基础四包构建和当前注册测试通过，建立对照基线。重构后：

| 检查 | 结果 |
|---|---|
| 默认二维构建 | core、nodes、bringup、RViz、pf_debug 五包通过 |
| 独立 FAST-LIO2 构建 | core（ON）、fast_lio、icp_relocalization、scurm_sim 四包通过 |
| 基础 core | 76 个实际用例通过，覆盖地图、膨胀、连续碰撞、规划、控制和 AMCL |
| 真实节点及节点 C++ | 27 个实际用例通过，含取消/抢占、动态障碍、输入失效、epoch 与独立速度守卫 |
| RViz 面板测试 | 1 个用例通过；本轮未做 GUI 人工验收 |
| FAST-LIO2 adapter | 24 个 pytest 用例通过 |
| 新 FAST-LIO2 计算回归 | 5 个用例通过：不完整输入、静止房间/双实例交错、无效先验、日志失败和快照预处理 |
| launch 解析 | 默认 AMCL、FAST-LIO2 `--show-args` 通过 |
| 隔离 Gazebo | 域 230 / `mini_nav_module_partition_20261006`，人工初值与运动中重定位验收通过 |

合计 **133 个实际用例**；统计只取当前注册目标，排除旧 XML 和 CTest 包装重复。pf_debug 没有注册单元测试，本轮验证其迁移后的构建。上游 Eigen/IKFoM 编译警告保留，不能将成功构建表述为全部警告已清除。

Gazebo 验证未初始化时拒绝导航并保持零速，拒绝错误帧及零四元数；人工初值后定位成功。运动中重新设定位姿，约 **0.0493 s** 后撤销运动许可；旧任务以 `localization_reset` 退出，旧 FAST-LIO2 子进程结束，重新定位不自动恢复旧目标。随后新目标成功到达，停车位姿约 `(0.02751, 0.88506, 1.66350)`，输出零速度，`/cmd_vel` 唯一发布者为 `velocity_guard`。测试 launch 与自身子进程已清理。

Git 留存证据：[测试计数](module_partition_evidence/current_test_counts.json)、[Gazebo 验收](module_partition_evidence/gazebo_acceptance.json)。原始运行与构建日志按仓库约定只保留在本地 `module_partition_evidence/`：`final_test.log`、`gazebo_acceptance.log`、`final_build.log`、`fastlio_build.log`，不加入 Git。

## 复现命令与文档更新

从工作区根目录执行：

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup mini_nav_rviz_plugins mini_nav_pf_debug \
  --cmake-args -DMINI_NAV_BUILD_FASTLIO2=OFF -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select mini_nav_core mini_nav_nodes mini_nav_rviz_plugins \
  --event-handlers console_cohesion+

# 正式独立后端构建脚本已包含 core、ON 开关和新路径。
bash src/mini_nav/mini_nav_fastlio/scripts/build_localization.sh
source install_mini_nav_fastlio/setup.bash
ros2 launch scurm_sim fastlio2_navigation.launch.py --show-args
```

本轮验收使用 `build_module_partition_fastlio` / `install_module_partition_fastlio` 独立输出，避免覆盖此前三维部署。显式 `--base-paths` 选择 core、fast_lio、icp_relocalization 和 scurm_sim；测试使用对应 build-base。基础构建继续使用正常 build/install。源码变化不会热替换已有运行进程。

更新 docs 的 architecture、localization_backends、contributing、project_status，以及 planning_costmap、local_costmap、navigation_tasks 中的源码引用；教学 HTML 仅更新源码链接，旧验证 JSON 保留为历史证据。本轮未重做这些 HTML 的浏览器交互验收。模块 README 和源码 SHA256 清单同步更新；旧讨论里的已迁移文件通过留档路径读取，避免误把调研状态解释成当前实现。

## 保留边界

ICP 独立包内部算法和 ROS 回调尚未逐函数拆分，本轮只迁移目录；不声称所有上游代码已完全纯算法化。没有实车、长期运行或全部噪声/地图组合验收，Gazebo 结果只覆盖上述有限场景。FAST-LIVO2 独立前端与其他工作区工程未改写。
