# AMCL 与 FAST-LIO2 定位源码

整理日期：2026-10-04。整理前的可运行版本已保存在工作区 Git 提交 `94b4189`，包含 SCURM 固定版本、现有 Jazzy/仿真补丁、`/initialpose` 接入和相关跟踪修复。未将构建、安装、日志或用户 PCD 加入提交。

## 目录与计算边界

2026-10-06 重构前完整留档为 `f2218c9`；此前源码整理基线 `94b4189` 仍保留。

```text
mini_nav_core/
├── include/mini_nav_core/localization/
│   ├── amcl/                         # AMCL 模型、粒子滤波、观测数据
│   └── fastlio2/
│       ├── fastlio_types.hpp          # 普通点云、IMU、配置和结果
│       ├── fastlio_estimator.hpp      # 串行单实例计算接口
│       └── pointcloud_preprocess.hpp  # 已解码点云预处理
└── src/localization/
    ├── amcl/
    └── fastlio2/                      # IMU、估计器、预处理、IKFoM、ikd-Tree
mini_nav_nodes/
├── include/mini_nav_nodes/
│   ├── amcl/amcl_node.hpp
│   └── fastlio2/
│       ├── fastlio_node.hpp
│       └── sensor_input.hpp
└── src/fastlio2/
    ├── fast_lio/src/
    │   ├── main.cpp
    │   └── node/
    │       ├── fastlio_node.cpp        # 实例、回调调度、先验地图装载
    │       ├── node_parameters.cpp     # ROS 参数声明与读取
    │       ├── sensor_input.cpp        # ROS 解码、队列与时间同步
    │       └── ros_output.cpp          # ROS 点云、位姿、TF、路径输出
    ├── icp_relocalization/             # 独立 ICP ROS 包，内部本轮未改写
    └── adapter/                        # 质量、TF、初值、会话与子进程管理
mini_nav_fastlio/
├── scurm_sim/                          # launch/config/models/test
├── scripts/
├── patches/                           # 先前的上游兼容补丁
└── SOURCE_MANIFEST.json
```

默认 `mini_nav_core` target 编译二维导航与 AMCL，不链接 PCL。`MINI_NAV_BUILD_FASTLIO2=ON` 额外生成 `mini_nav_core::mini_nav_fastlio2_core`，依赖 Eigen/PCL/OpenMP，不依赖 ROS。`fast_lio` 节点通过公开头文件和导出 target 使用它，不访问 core 私有头文件。ROS 消息解码为 `ImuSample`、PCL 点云和 `LivoxScan` 后才送入计算层。

`FastlioEstimator` 持有实例自己的滤波器、地图索引与状态。调用者串行提供一帧点云及覆盖该帧的 IMU；结果中的点云借用内部缓冲区，必须在下一次 `Process` 前使用。重新定位仍由 adapter 重启会话和估计器，`SetInitialPose` 用于首次先验初始化，不是在线重置整个滤波器的接口。

源码来源为 `PolarisXQ/SCURM_SentryNavigation@46e6425c692ec98f8e65446fb6fdd360f44ef8e5`。保留原注释与许可，不将其描述为自研 FAST-LIO2。拆分中将 IKFoM 观测回调改为实例绑定，修复有效点云清空后越界写入和日志文件打开失败后的写入保护。清单的 `checkpoint_sha256` 保留先前归档来源，`sha256` 对应当前文件；拆分文件记录原文件出处。FAST_LIO 保留 GPL-2.0 LICENSE 和文件级声明，ICP 包声明 Apache-2.0。

## 构建与运行

AMCL 原入口：

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup
source install/setup.bash
ros2 launch mini_nav_bringup mini_localization_astar.launch.py
```

FAST-LIO2 先验定位：

```bash
cd /home/a/ros2_ws
bash src/mini_nav/mini_nav_fastlio/scripts/build_localization.sh
source install_mini_nav_fastlio/setup.bash
ros2 launch scurm_sim fastlio2_navigation.launch.py
```

默认 PCD 与二维地图沿用既有文件；构建脚本第一个参数可指定同场景 PCD。正常构建直接使用已归档的源码、仿真模型与配置，不克隆或重新施加补丁。已有 `install_fastlivo` overlay 只提供 Livox 消息构建依赖，运行不启动 LIVO2 节点。

三维后端使用 `build_mini_nav_fastlio` / `install_mini_nav_fastlio`，构建日志写入 `log_scurm`；直接 launch 的运行日志由 `ROS_LOG_DIR` 控制，未设置时使用 ROS 默认日志目录。原 `build_scurm` / `install_scurm` 保留，迁移时不覆盖正在运行的二进制。`mini_nav_fastlio/COLCON_IGNORE` 防止主工作区自动发现此独立部署；构建通过 `--base-paths` 显式发现 core、两个节点包和 `scurm_sim`；脚本传入 `MINI_NAV_BUILD_FASTLIO2=ON`。默认导航构建使用 OFF，二维运行不需要三维依赖。

`src/scurm_deploy` 保留为指向 `mini_nav/mini_nav_fastlio` 的兼容链接，原 shell 命令仍有效。ROS 包名 `fast_lio`、`icp_relocalization`、`scurm_sim` 和 ROS 接口保持不变。正式导航入口为 `scurm_sim/fastlio2_navigation.launch.py`；原 `mini_nav_fastlio.launch.py` 作为兼容入口包含它。

建图运行工具为 `build_fastlio.sh` / `run_fastlio.sh`，保存地图为 `save_pcd.sh`。它们与定位入口共享归档源码。`prepare.py` / `configure.py` / `build.sh` / `run_sim.sh` 及忽略的 `upstream/` 是此前整套上游 Nav2 实验工具；当前 mini_nav 定位入口不调用这些准备工具。

## 直接 launch 参数

新入口设置全部子进程的 ROS / Gazebo 环境，启动不需要 `run_localization.sh`，也不需要提前 export 域或分区。`install_mini_nav_fastlio/setup.bash` 已串联 Jazzy、Livox 消息依赖和主工作区环境。默认 `ros_domain_id:=227`、`gz_partition:=scurm_mini_nav`、`rmw_implementation:=rmw_cyclonedds_cpp`、`gui:=true`、`rviz:=true`。同域同分区已有实例时，先关闭原 launch；Ctrl+C 关闭新 launch 及其后端子进程。

隔离运行示例：

```bash
ros2 launch scurm_sim fastlio2_navigation.launch.py \
  ros_domain_id:=226 gz_partition:=mini_nav_fastlio_direct_launch \
  gui:=false rviz:=false
```

原入口沿用已有环境值作为隔离参数默认值；新入口用上面的固定默认值，并允许显式覆盖。

## 初始化与边界

定位使用 `ROS_DOMAIN_ID=227`、`GZ_PARTITION=scurm_mini_nav`；建图为 231 / `scurm_fastlio2_demo`。默认 AMCL 的域仍为 61 / `mini_nav`。

RViz 使用 **2D Pose Estimate** 发布 `/initialpose`，配准有效后用 **2D Goal Pose** 导航。再次设置位姿会使旧任务失效，ICP / FAST-LIO2 重新配准后需提交新目标。邻域 ICP 需要接近实际位置的初值，未实现未知位置全局重定位。

质量与 epoch 接入保持原行为，定位动态发布 `map → odom`；`velocity_guard` 是定位导航入口唯一的 `/cmd_vel` 发布者。迁移前验收见 [人工初始化记录](../logs/26-10-4/scurm_fastlio2_initialpose.md)，此前整理验收见 `logs/26-10-4/scurm_source_reorganization.md`，本轮拆分与验证见 [重构实施记录](../logs/26-10-6/core_node_module_partition_implementation.md)。
