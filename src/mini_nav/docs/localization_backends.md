# AMCL 与 FAST-LIO2 定位源码

整理日期：2026-10-04。整理前的可运行版本已保存在工作区 Git 提交 `94b4189`，包含 SCURM 固定版本、现有 Jazzy/仿真补丁、`/initialpose` 接入和相关跟踪修复。未将构建、安装、日志或用户 PCD 加入提交。

## 目录

```text
mini_nav/
├── mini_nav_core/
│   ├── src/localization/
│   │   ├── amcl/                 # 原六个 AMCL 实现文件
│   │   └── fastlio2/             # IKFoM、过程模型、ikd-Tree、旋转数学
│   └── include/mini_nav_core/localization/  # AMCL 公开头文件保持原路径
├── mini_nav_nodes/
│   ├── src/amcl_node.cpp         # 原 AMCL ROS 节点
│   └── src/localization/fastlio2/
│       ├── fast_lio/             # ROS IMU/点云处理、点面观测、FAST-LIO2 节点
│       ├── icp_relocalization/   # 初值邻域 ICP ROS 包
│       └── adapter/              # 定位接入、质量/TF、会话与子进程管理
└── mini_nav_fastlio/
    ├── scurm_sim/                # 原包名保留，launch/config/models/test
    ├── scripts/                  # 构建、运行、保存 PCD、验收工具
    ├── patches/                  # 整理前已验证的上游兼容补丁
    └── SOURCE_MANIFEST.json      # 固定提交、迁移位置和文件 SHA256
```

`mini_nav_core` 基础库仍显式编译 AMCL 和二维导航算法，不链接 ROS、PCL 或 FAST-LIO2。FAST-LIO2 的独立 `fast_lio` ROS 包显式引用 `core/src/localization/fastlio2` 内核。上游算法源文件、注释和命名保持原样，修改集中在构建、安装与路径引用；没有把 FAST-LIO2 改写成自研算法。

源码来源为 `PolarisXQ/SCURM_SentryNavigation@46e6425c692ec98f8e65446fb6fdd360f44ef8e5`。FAST_LIO 的 GPL-2.0 `LICENSE` 和源码中的其他许可声明保留；ICP 包的 `package.xml` 声明 Apache-2.0。清单中的 `checkpoint_sha256` 对应整理前留档，`sha256` 对应当前位置的文件内容。

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

三维后端使用 `build_mini_nav_fastlio` / `install_mini_nav_fastlio`，构建日志写入 `log_scurm`；直接 launch 的运行日志由 `ROS_LOG_DIR` 控制，未设置时使用 ROS 默认日志目录。原 `build_scurm` / `install_scurm` 保留，迁移时不覆盖正在运行的二进制。`mini_nav_fastlio/COLCON_IGNORE` 防止主工作区自动发现此独立部署；构建通过 `--base-paths` 显式发现两个节点包和 `scurm_sim`。

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

质量与 epoch 接入保持原行为，定位动态发布 `map → odom`；`velocity_guard` 是定位导航入口唯一的 `/cmd_vel` 发布者。迁移前验收见 [人工初始化记录](../logs/26-10-4/scurm_fastlio2_initialpose.md)，本轮整理验收另存于 `logs/26-10-4/scurm_source_reorganization.md`。
