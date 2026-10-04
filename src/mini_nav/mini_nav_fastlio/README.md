# mini_nav 的 SCURM FAST-LIO2 后端

源码已按计算内核、ROS 节点及仿真工具整理，详见 [目录与构建说明](../docs/localization_backends.md)。旧 `src/scurm_deploy` 是指向本目录的兼容链接。

上游：[四川大学火锅战队 SCURM_SentryNavigation](https://github.com/PolarisXQ/SCURM_SentryNavigation)，固定提交 `46e6425c692ec98f8e65446fb6fdd360f44ef8e5`。

当前可运行入口为 **FAST-LIO2 三维建图演示**：Gazebo Harmonic 的 Waffle 模型带 720×32 三维 GPU 雷达（10 Hz）和 IMU（200 Hz），SCURM FAST-LIO2 输出估计位姿、点云地图和轨迹。未启动 AMCL、FAST-LIVO2、先验 ICP 或 Nav2 导航器。

```bash
cd /home/a/ros2_ws
bash src/mini_nav/mini_nav_fastlio/scripts/build_fastlio.sh
bash src/mini_nav/mini_nav_fastlio/scripts/run_fastlio.sh
```

默认打开 Gazebo、RViz 与中文控制窗口。**按住** W/A/S/D 或按钮移动，松开停止，空格停止；窗口失焦也停止。直行/倒退时检查扫描前后净空，速度受适配器限幅和输入超时保护。这是人工建图演示，不是完整自主避障控制。

停止本演示：

```bash
bash /home/a/ros2_ws/src/mini_nav/mini_nav_fastlio/scripts/stop_fastlio.sh
```

RViz 的 `FAST-LIO2 Map` 显示 `/ikd_tree`，`Current Registered Scan` 显示 `/cloud_registered`，`FAST-LIO2 Trajectory` 显示 `/path`。固定坐标系为 `odom`，原点来自本次 FAST-LIO2 会话。移动鼠标旋转、平移和缩放可查看三维点云。没有用静态 map→odom 代替定位。

保存当前累计地图，不需要停止或重启建图：

```bash
bash /home/a/ros2_ws/src/mini_nav/mini_nav_fastlio/scripts/save_pcd.sh
```

每次保存生成 `maps/scurm_fastlio2/map_<时间>/map.pcd` 和 `metadata.json`，不会覆盖已有地图。PCD 为二进制 XYZ + intensity，坐标系为本次会话 `odom`，元数据包含消息时刻、点数、范围和 SHA256。保存的是最新一帧 `/ikd_tree` 累计地图，不是单帧 `/cloud_registered`；RViz 的高度着色是显示方式，PCD 保留几何及 intensity。此入口没有开启原生 `/map_save` 的累计显示云保存，因此使用独立订阅导出。

运行隔离：`ROS_DOMAIN_ID=231`，`GZ_PARTITION=scurm_fastlio2_demo`，CycloneDDS，本脚本使用 `log_scurm/fastlio_runtime/launch.lock` 防止重复启动。所有运行节点使用仿真时钟。建图不需要预置二维 YAML/PGM，用户原地图保持不变。

构建使用 `build_mini_nav_fastlio` / `install_mini_nav_fastlio` / `log_scurm`，仅构建 `fast_lio` 与 `scurm_sim`。`install_fastlivo` overlay 提供已有 Livox 消息依赖，**不会启动其中的 FAST-LIVO2 前端**。这台机器已有 ROS Jazzy、Gazebo/ROS 桥、PCL、TurtleBot3、NumPy、SciPy、Tkinter 和 RViz；仿真模型、配置和适配代码已经归档，正常构建不再从 FAST-LIVO2 模型生成。

## 上游兼容补丁

正常建图/定位构建直接使用 mini_nav 内已归档的源码，不再联网克隆或调用补丁生成器。`SOURCE_MANIFEST.json` 记录来源、文件位置与哈希，历史兼容差异保存在 `patches/jazzy_sim.patch`。`scripts/prepare.py`、`configure.py` 和忽略的 `upstream/` 仅保留为此前整套上游导航实验的准备工具，不是当前定位构建的源码来源。适配包括 C++17/TF 依赖、仿真 IMU QoS、有限点过滤、仿真快照类型、正确的带符号 IMU 时间传播、可禁用前端 TF、一次完整里程计发布与轨迹头时间戳。

Gazebo GPU 雷达在同一仿真时刻生成整个点云，没有真实旋转雷达的逐点采样时间。因此仿真快照保留 IMU 状态传播和原点面迭代滤波、ikd-tree 建图，跳过不适用的逐点运动去畸变；不会伪造扫描内时间。当前结果不能代表 MID360 的实物时序、标定与扫描畸变表现。

地图显示使用当前 ikd-tree，可随探索增长并下采样，避免反复累积整帧显示云。轮式仿真里程计只供独立验证，不输入 FAST-LIO2，也不发布定位 TF；`odom→base_footprint` 由适配器根据 FAST-LIO2 的 IMU 位姿和真实安装外参生成，`/cmd_vel` 也只有该适配器发布。

旧的 `bringup.launch.py` 与 `build.sh` 是此前准备的整套先验导航入口，尚未完成运行验收；本次请使用上述 `run_fastlio.sh` / `build_fastlio.sh`。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install_fastlivo/local_setup.bash
source /home/a/ros2_ws/install/local_setup.bash
source /home/a/ros2_ws/install_mini_nav_fastlio/local_setup.bash
export ROS_DOMAIN_ID=230
colcon --log-base log_scurm/fastlio_test test --build-base build_mini_nav_fastlio \
  --install-base install_mini_nav_fastlio --base-paths src/mini_nav/mini_nav_fastlio/scurm_sim \
  --packages-select scurm_sim --event-handlers console_cohesion+
colcon test-result --test-result-base build_mini_nav_fastlio/scurm_sim --verbose
```

运行时只读检查，可在演示运行期间另开终端执行：

```bash
export ROS_DOMAIN_ID=231
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
python3 src/mini_nav/mini_nav_fastlio/scripts/validate_fastlio.py \
  --output /tmp/scurm_fastlio_streams.json
```

`--motion --controls-pid <本演示控制窗口PID>` 是人工选用的短距离验证，会暂时暂停该控制窗口，在前/后更空的一侧做有限平移，再原地转向，结束后恢复窗口。此检查对比 FAST-LIO2 与轮式里程计的相对运动，不能作为绝对真值精度测试。

## FAST-LIO2 先验定位接入 mini_nav

使用火锅战队的 ICP 初始化和 FAST-LIO2 固定先验地图模式，导航仍由 mini_nav 自己的 A*、路径跟踪、任务管理和速度保护完成。此入口不启动 AMCL、FAST-LIVO2 或 Nav2 规划/控制服务器。

```bash
cd /home/a/ros2_ws
bash src/mini_nav/mini_nav_fastlio/scripts/build_localization.sh
bash src/mini_nav/mini_nav_fastlio/scripts/run_localization.sh
```

默认使用已保存的 `maps/scurm_fastlio2/map_20261004_190840_437173/map.pcd`，二维地图来自 `src/mini_nav/mini_nav_bringup/maps/tb3_learning.yaml`。可将同一仿真初始坐标系的其他 PCD 作为构建脚本的第一个参数；脚本检查局部对齐误差并生成独立先验，不修改原地图。场景初始位置固定为 Gazebo `(-2, -0.5, 0)`，当前初始化是邻域 ICP，不能当作未知位置的全局重定位。

定位导航使用 `ROS_DOMAIN_ID=227`、`GZ_PARTITION=scurm_mini_nav`；建图演示仍使用 231 / `scurm_fastlio2_demo`。先在 RViz 中用 **2D Pose Estimate** 点选位置、拖动箭头指定朝向，发送 `/initialpose`；ICP 和 FAST-LIO2 配准成功后，再用 **2D Goal Pose** 发送 `/goal_pose`，也可发送 `/navigate_to_pose` Action。刚启动该仿真时，地图中的初始位置约 `(0,0)`，朝向 `+X`。初始猜测仍需接近真实位姿。

启动时不再预置 ICP 初值。运行中再次发送 `/initialpose` 会立即撤销旧任务，重启 ICP / FAST-LIO2 并建立新配准会话；成功后需要重新发送导航目标，不需要手工重启 Gazebo。消息类型为 `geometry_msgs/msg/PoseWithCovarianceStamped`，`header.frame_id` 必须是 `map`，位姿代表 `base_footprint`；接入层负责 IMU 外参转换。非法坐标系或四元数不会启动新定位后端。

```bash
bash src/mini_nav/mini_nav_fastlio/scripts/stop_localization.sh
```

`/scurm/localization_status` 给出有效性、失效原因、匹配指标及数据年龄。质量丢失会更新 `/mini_nav/localization_epoch`，终止旧任务；恢复后需要新目标。`map → odom` 由时间对齐的 FAST-LIO2 位姿与轮式里程计动态计算，`velocity_guard` 是唯一 `/cmd_vel` 发布者。该实验启用 0.03 m 的终点转向切换缓冲，保留 0.12 m 的到达容差；其他入口默认缓冲为零。

人工初始化与运行中重新配准的验证见 [initialpose 接入记录](../logs/26-10-4/scurm_fastlio2_initialpose.md)。此前往返、定位中断停车、回归测试及已知限制见 [部署验证记录](../logs/26-10-4/scurm_fastlio2_mini_nav_localization.md)。
