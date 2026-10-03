# FAST-LIVO2 Jazzy / Gazebo 部署

本目录已经在本机 ROS 2 Jazzy、Gazebo Harmonic 上完成构建、静止初始化和短程运动验收。
使用现有 TurtleBot3 Waffle 场景，新增真正的三维 GPU LiDAR，启用 LiDAR + IMU + 相机融合。
此目录提供相对里程计前端。mini_nav 已新增独立的建图/旧图导航入口，使用人工初值和三维先验配准，见 [双入口使用说明](../mini_nav/mini_nav_fastlivo/README.md)；下方原入口继续作为前端与三维显示建图工具保留。

详细结果：[部署报告](../mini_nav/logs/26-10-2/fastlivo2_gazebo_deployment.md)、
[彩色建图点保留与存盘](../mini_nav/logs/26-10-3/fastlivo2_persistent_color_mapping.md)。

## 在本机启动

现有安装已可直接运行，不需要先重新构建：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash
source /home/a/ros2_ws/install_fastlivo/setup.bash
export ROS_DOMAIN_ID=219
export GZ_PARTITION=mini_nav_fastlivo_deploy
ros2 launch fastlivo_sim bringup.launch.py map_dir:=/home/a/ros2_ws/maps/fastlivo2
```

也可使用 `bash /home/a/ros2_ws/src/fastlivo2_deploy/scripts/run_sim.sh`，
脚本会设置相同环境并提供重复启动文件锁。直接 `ros2 launch` 时只启动一套实例。

默认由 `fastlivo_sim/bringup.launch.py` 同时打开 Gazebo 和 RViz，
启动 Waffle、传感器桥接、FAST-LIVO2、两个累计地图节点和 PS5 控制节点。
PS5 控制初始关闭，使用 RViz 的“PS5 手柄控制”面板或 L1 启用。
两个窗口随同一 launch 管理；在启动终端 Ctrl+C 结束整套运行并保存地图。
本机 RViz 14.1.22 在接收仿真数据后退出时仍可能报错或等待约 5 秒后被 launch 终止；
实际验收中地图节点正常保存、Gazebo 正常退出，最终没有残留子进程。
该系统 RViz 组件异常尚未修复，详细证据见
[合并启动记录](../mini_nav/logs/26-10-3/gazebo_rviz_combined_launch.md)。
无需再单独运行 `view.sh`。可按需关闭窗口：

```bash
# 只打开 Gazebo
bash src/fastlivo2_deploy/scripts/run_sim.sh rviz:=false
# 无界面运行
bash src/fastlivo2_deploy/scripts/run_sim.sh gui:=false rviz:=false
```

只启动一次；脚本用原有文件锁防止重复启动。
`frontend`、`mapping`、`map_dir` 参数继续支持，也可用 `rviz_config:=/路径/配置.rviz` 指定预设。
`teleop:=false` 可关闭手柄节点，供其他控制入口使用；手柄与其他控制器不能重复启动。
当前已有旧启动进程运行时，先在其终端 Ctrl+C 退出再使用新入口。
等待几秒静止初始化，然后在另一终端查看融合位姿：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install_fastlivo/setup.bash
export ROS_DOMAIN_ID=219
export GZ_PARTITION=mini_nav_fastlivo_deploy
export ROS_LOG_DIR=/home/a/ros2_ws/log_fastlivo/runtime
ros2 topic echo /aft_mapped_to_init --once
```

`/aft_mapped_to_init` 是最终融合的 IMU 位姿，父坐标为 `camera_init`，子坐标为 `aft_mapped`。
它不直接代表 `map -> base_footprint`，也没有发布 `map -> odom`。
`/cloud_registered` 是同一状态时间下、经过最终位姿投影的相机视野内彩色点云。

## 验收命令

新入口默认已经打开 RViz。对于只启动服务或关闭过 RViz 的情况，
保持仿真运行，在另一终端单独打开预设：

```bash
bash /home/a/ros2_ws/src/fastlivo2_deploy/scripts/view.sh
```

预设使用 `camera_init` 坐标系，显示 `/fastlivo/color_map` 累计彩色地图、`/cloud_registered` 当前彩色点云、
`/path` 估计轨迹、`/aft_mapped_to_init` 当前位姿及 `/rgb_img` 视觉图像。
累计地图按 0.1 m 体素保留已观测区域；当前彩色点云只保留 0.5 秒作为动态观察层。
RViz 的 Displays 面板提供两个独立的勾选开关：

- `LiDAR map (rainbow)`：累计激光雷达地图，按高度显示彩虹色，默认关闭，勾选开启。
- `Camera map (RGB)`：累计摄像机彩色地图，保留原始 RGB，默认开启。

开关只控制显示，两个地图节点仍持续累积和存盘；可以单独开启，也可以同时显示。
两层叠加时，高度着色可能遮住摄像机的原始颜色。已有 RViz 窗口仍使用旧名称，重新打开预设可加载新名称。
观察静止时轨迹是否明显漂移；运动时轨迹是否连续、点云中的墙面是否稳定且没有明显重复。
Gazebo 显示的机器人运动可供直观对照；量化误差按下方验收命令计算。

## 保留三维地图

默认 `mapping:=true`，独立地图节点接收 `/fastlivo/cloud_world`：这是使用最终融合位姿投影的
完整 360° 三维几何扫描，包含相机视野外的有效点。该节点不向定位器反馈地图或真值。
同一启动文件还启用独立的 `fastlivo_color_map_store`，使用 `colored:=true` 接收 `/cloud_registered`，
保留其 XYZ 和摄像机 RGB 颜色，并以 `/fastlivo/color_map` 发布完整累计彩色地图。

- 地图以 `/fastlivo/map` 发布，Reliable + Transient Local，重新打开 RViz 可获得完整当前地图。
- 每 15 秒模拟时间和正常退出时自动保存，PCD 文件原子替换。
- 每次运行新建独立目录：`/home/a/ros2_ws/maps/fastlivo2/<时间戳和随机后缀>/map.pcd`。
- 彩色地图存入独立的 `<时间戳>_color_<随机后缀>/color_map.pcd`；字段是 XYZ + packed RGB。
- 同目录的 `metadata.json` 记录坐标系、采样时间、扫描数、体素尺寸、颜色模式和地图容量状态。
- 默认最多保留 100 万个体素；达到限制会警告并停止新增体素，已有地图不会被删除。
- 仿真重启必须整套重启。时钟倒退或输入坐标系改变时，地图节点保留旧图并停止合并。

立即存盘可在设置好域 219 的终端调用：

```bash
ros2 service call /fastlivo/save_map std_srvs/srv/Trigger '{}'
ros2 service call /fastlivo/save_color_map std_srvs/srv/Trigger '{}'
```

返回值包含实际保存路径。启动时可传 `map_dir:=/所需目录` 改变存储位置，
或传 `mapping:=false` 关闭累计地图节点。退出时用 Ctrl+C，让节点保存最后一批数据。
几何 PCD 是 XYZ/intensity，彩色 PCD 是 XYZ/rgb；每个体素保留首次有效观测的位置和颜色，
后续观测不覆盖它。这是雷达几何与摄像机颜色融合后的点云，不是图像里的视觉特征标记，
也没有保存完整视觉稀疏地图或滤波器状态。
当前可以边建图边估计相对位姿；重启后加载旧 PCD 重定位、回环和全局地图优化仍未接入。
此前关闭保存且已经结束的运行，不能从有限历史显示中恢复完整旧地图。

在上面的第二终端环境中执行。默认只观测，不发送非零速度：

```bash
ros2 run fastlivo_sim probe --output /home/a/ros2_ws/log_fastlivo/manual_static --seconds 15
```

可选运动验收：5 秒静止、10 秒以 0.05 m/s 直行、停顿、4 秒以 0.15 rad/s 转向，最后停车。
该命令仅允许在指定的仿真域和分区执行，每次完整验收应重启场景回到初始位置：

```bash
ros2 run fastlivo_sim probe --output /home/a/ros2_ws/log_fastlivo/manual_motion --seconds 25 --drive
python3 /home/a/ros2_ws/src/fastlivo2_deploy/scripts/evaluate.py \
  /home/a/ros2_ws/log_fastlivo/manual_motion/result.json
```

先等待 FAST-LIVO2 完成静止初始化再做运动验收。验收程序退出时发送零速度。
此程序用于仿真前端验收，生产导航仍应通过 mini_nav 的 `velocity_guard_node` 发出速度。

## PS5 USB 手柄控制

新总入口已经启动手柄节点，RViz 的 **PS5 手柄控制** 面板可以直接管理：

- 点击“启用控制”或按 L1 开启，蓝灯、短振动。
- 点击“关闭控制 / 停车”或再次按 L1 关闭，红灯、短振动。
- 设置线速度上限和角速度上限，点击“应用速度上限”；范围分别为 0–2 m/s、0–2 rad/s。
- 面板显示 USB 连接、实际控制状态和已应用的限速。

旧仿真或以 `teleop:=false` 启动的仿真，可在另一个终端执行：

```bash
bash /home/a/ros2_ws/src/fastlivo2_deploy/scripts/teleop_ps5.sh
```

左摇杆上下控制前后，右摇杆左右控制转向；上限分别为 2 m/s、2 rad/s。
按一次 L1 启用（蓝灯、短振动），再按一次关闭并停车（红灯、短振动）。
手柄断连或输入超时后关闭控制，恢复后需重新按 L1。Ctrl+C 停止手柄控制进程。
该入口复用用户的 DualForge USB 驱动，通过原有速度保护节点输出 `/cmd_vel`。
启动时检查速度发布者冲突，不能与运动验收程序或自主导航同时控制机器人。
新完整入口已经启动控制器时，不再单独运行该脚本。
依赖安装、构建与参数说明见 [mini_nav_teleop](../mini_nav_teleop/README.md)。

## 重建和复现源码

所有上游仓库固定在 `repos.lock.json` 中，Jazzy 兼容改动保存在 `patches/`。
x86 构建保留系统基线指令集，避免 `-march=native` 与系统 PCL/Eigen 的内存对齐 ABI 不一致导致退出崩溃；
地图节点在关闭 ROS context 前完成当前回调和最后一次存盘。
上游下载目录被 Git 忽略，避免把嵌套仓库误加入 mini_nav 的 Git 历史。

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_teleop
python3 src/fastlivo2_deploy/scripts/prepare.py
bash src/fastlivo2_deploy/scripts/build.sh
bash src/fastlivo2_deploy/scripts/test.sh
```

`prepare.py` 在缺失时按固定提交下载源码；对已有不同版本或补丁冲突会报错并保留目录。
本机已下载时不会重新访问网络。Sophus 安装到工作区内部，不修改系统依赖。

依赖本机已有的 Eigen3、PCL、OpenCV、fmt，以及 Jazzy 的 rclcpp、pcl_ros、pcl_conversions、
cv_bridge、image_transport、ament_cmake_gtest、ros_gz_sim、ros_gz_bridge、ros_gz_image 和 turtlebot3_gazebo。
分析脚本另用 NumPy、SciPy。本轮没有运行系统软件安装。

Livox 包仅编译原始 ROS 消息接口：`BUILD_LIVOX_DRIVER=OFF`。未安装 Livox 硬件 SDK，
不能把这个安装当作可直接连接实机雷达的 Livox 驱动。

顶层 `COLCON_IGNORE` 避免本目录进入平时的默认全工作区构建；专用脚本显式指定各个包。
产物使用 `build_fastlivo/`、`install_fastlivo/`、`log_fastlivo/`，保留原 `build/install/log`。

## 仿真输入约定

- 3D LiDAR：720 × 32、360° 水平、±20° 垂直、10 Hz，`/fastlivo/lidar/points`。
- 相机：640 × 480、10 Hz，`/camera/image_raw`，内参来自 Gazebo CameraInfo 核验。
- IMU：200 Hz，`/imu`；现有 `/scan` 二维雷达继续保留。
- `preprocess.lidar_type=8` 专门表示 Gazebo 瞬时三维快照，各点时间等于消息头时间。
  真实旋转扫描雷达必须使用对应的数据格式和逐点时间处理。

模型使用显式 IMU 安装位置及新增 LiDAR 位置；当前外参只适用于这个 SDF。
没有把二维 `/scan` 转换成假的三维输入，也没有给 FAST-LIVO2 使用 Gazebo 真值作为定位输入。
`/fastlivo/ground_truth` 仅被验收程序订阅用于误差计算。

## 本机启动兼容项与限制

本机 `LD_LIBRARY_PATH` 含 `/opt/MVS`，其旧 libusb 会导致 PCL 出现
`undefined symbol: libusb_set_option`。launch 仅为 FAST-LIVO2 子进程预加载系统 libusb；
不会修改全局环境或 MVS 文件。

`bringup.launch.py` 已完成 Gazebo/RViz 窗口启动、轨迹及两种累计地图消息的隔离域验收。
位姿协方差仍沿用上游未填充的输出，不能将全零协方差解释为高置信度。
长距离、传感器掉线、时钟回退和重定位尚未完成导航级验收。
