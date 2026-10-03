# FAST-LIVO2 建图与旧图导航

当前实现面向本机 ROS 2 Jazzy 和现有平地 Waffle Gazebo 场景。新增独立包，不复制导航核心；AMCL 原入口继续保留。实际验收与限制见 [实施报告](../logs/26-10-3/fastlivo_two_launch_implementation.md)。

## 准备环境

FAST-LIVO2 和 `fastlivo_sim` 使用已经部署的 `install_fastlivo` overlay，部署方式见 [部署说明](../../fastlivo2_deploy/README.md)。配准依赖固定为 `small_gicp==1.0.1`，锁定本机 CPython 3.12、Linux x86_64 wheel 及 SHA256，安装到 workspace 内部，不覆盖系统 Python。

```bash
cd /home/a/ros2_ws
bash src/mini_nav/mini_nav_fastlivo/tools/build.sh
```

若改动 `fastlivo_sim`，它被外层 `COLCON_IGNORE` 隔离，需要显式构建：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_fastlivo/setup.bash
colcon --log-base log_fastlivo build --base-paths src/fastlivo2_deploy/fastlivo_sim \
  --build-base build_fastlivo --install-base install_fastlivo --packages-select fastlivo_sim
```

每个操作终端都执行：

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_fastlivo/setup.bash
export ROS_DOMAIN_ID=219 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export GZ_PARTITION=mini_nav_fastlivo_deploy
```

先退出已有仿真、导航和手柄控制进程。两个入口会检查同域的地图、命令及仿真发布者冲突。

## 离线概率重放实验（2026-10-04）

当前实验保留原在线预览，建图入口默认 `record_session:=true`，另外保存完整去畸变 IMU 点云和严格同时间戳融合位姿。停止建图后再生成最终候选地图；在线预览和原 `/fastlivo/save_nav_map` 服务仍是原始布尔累积图，不能当成已优化结果。

先准备独立后端（仅安装到工作区，源码版本锁定）：

```bash
bash src/mini_nav/mini_nav_fastlivo/offline/build.sh
bash src/mini_nav/mini_nav_fastlivo/tools/build.sh
```

建图时可从日志或 `/fastlivo/mapping_status` 的 `recording_session` 字段找到本次 session。停车、退出建图后执行：

```bash
ros2 run mini_nav_fastlivo finalize_map /实际记录目录/session_xxxxxxxxxxxxxxxx \
  --output /home/a/ros2_ws/maps/fastlivo2_offline \
  --binaries /home/a/ros2_ws/build_mapping_offline/bin
```

输出 JSON 中 `bundles.original_probability` 是本次候选图，交给原导航 launch 的 `map_bundle` 即可。每次生成新目录；原记录、原地图不覆盖。默认方案重放完整3D扫描的真实射线，按帧去重 hit/miss，保留未确认命中为未知，再投影 0.02–0.40m 车高；二维激光噪声命中不写入最终静态墙。体素面积投影保护格边低障碍。

`--backend hba` 显式运行官方 HBA 的无 ROS1 适配并输出两个对照包；第一轮本机 A/B 中 HBA 未改善墙厚，因此默认不启用。HBA 使用下采样关键帧做优化，最终仍重放全部完整原扫描，不使用粗优化图或相机裁剪彩图作为碰撞地图。

边界：只验证了当前单层平地 Gazebo。自由投影沿用明确的平地观测假设，不证明整个车高柱每个高度都被看到。0.02m 三维体素、0.05m 二维格仍有量化包络；前端长期漂移、全局闭环、动态物体离线剔除和实车尚未验证。高度下界2cm与地面容差8mm绑定本机标定，换设备需重新验证；不缩小0.26m硬安全距离掩盖地图错误。

## 1. 建图，不运行自主导航

```bash
ros2 launch mini_nav_bringup fastlivo_mapping.launch.py \
  output_dir:=/home/a/ros2_ws/maps/fastlivo2
```

同时打开 Gazebo、RViz、FAST-LIVO2 和 PS5 控制。PS5 左摇杆前后、右摇杆转向；L1 切换控制开关，开启蓝灯、关闭红灯并短振动。RViz 的 PS5 面板管理控制和速度，速度上限沿用 2 m/s、2 rad/s。建议先低速观察建图质量。

RViz 保留摄像机彩色累积点云和雷达彩虹点云开关；二维高度地图新增 `/fastlivo/map_2d_live`。几何点云用于三维定位；距地面 0.02–0.40 m 的障碍投影为二维占用，真实 `/scan` 射线提供平地自由空间证据，未观测区域保持未知。

在另一个已加载上述环境的终端保存：

```bash
ros2 service call /fastlivo/save_nav_map std_srvs/srv/Trigger '{}'
```

返回成功消息中的目录，例如 `.../map_0123456789abcdef`。每次保存创建新目录，不覆盖旧地图。该目录同时包含 `geometry.pcd`、`navigation.yaml/pgm`、观测缓存、标定/坐标关系和哈希清单。

保存后在启动终端按 Ctrl+C，退出建图。彩色/彩虹显示地图的另存文件只是可视化资产，导航加载的是该服务返回的完整地图目录。

## 2. 重启后用旧地图导航

```bash
ros2 launch mini_nav_bringup fastlivo_navigation.launch.py \
  map_bundle:=/home/a/ros2_ws/maps/fastlivo2/map_0123456789abcdef
```

将示例目录替换成上一步返回的实际目录。导航入口读取旧的二维地图和三维定位地图，不启动全局地图累积/保存节点，也不启动 AMCL。

1. RViz 用 **2D Pose Estimate** 在旧地图上指定机器人当前大致位置和朝向。
2. 等待定位有效、局部地图有效，再用 **2D Goal Pose** 发送目标。
3. 原有导航状态面板显示任务反馈和取消入口。

```bash
ros2 topic echo /mini_nav/localization_state
```

定位使用真实同步的点云/融合位姿，GICP 将当前三维观测约束到旧图，再发布动态 `map -> odom`。这是有人工初值的局部重定位；没有实现全局地点搜索。失效、前端重置或重新指定初值会撤销旧任务。恢复后需重新设初值和目标，不会自行续跑。

本机已验收的示例地图可直接试用：

```bash
ros2 launch mini_nav_bringup fastlivo_navigation.launch.py \
  map_bundle:=/home/a/ros2_ws/maps/fastlivo2_acceptance/map_f321189dd1d14011
```

该图只覆盖测试短程区域；默认出生位置约在地图原点附近，仍需手动指定初始位姿。

## 参数和边界

- 两入口共有 `gui`、`rviz`、`ros_domain_id`、`gz_partition`、`x_pose`、`y_pose`、`yaw`；默认出生点为 `(-2.0, -0.5, 0)`。
- 建图默认 `teleop:=true`。无手柄仅观察可用 `teleop:=false`。测试外部命令源时用 `teleop:=false external_control:=true`，外部源必须同时发布原始命令与任务租约，不能直发 `/cmd_vel`。
- `use_simulator:=false` 仅用于已有完整传感器/FAST-LIVO 前端运行时的接入，当前仍采用本仿真的标定与平地参数，没有验收实车。
- 高度、分辨率、容量见 `config/mapping.yaml`。保存后的高度区间和 IMU 到车体外参随图绑定。当前外参 `[0.032, 0, -0.078]`，换设备必须重新标定和建图。
- 导航期间 FAST-LIVO2 仍维护内部局部几何/视觉参考，已保存的全局地图不变；没有实现从旧图恢复完整 FAST-LIVO 视觉状态。
- 自由空间采用平地真实二维射线模型，不等价于整个车体高度柱都可通行。坡道、台阶、悬空障碍、负障碍和重复走廊全局误匹配尚未验收。

## 验证

```bash
colcon test --packages-select mini_nav_fastlivo mini_nav_nodes mini_nav_bringup \
  --event-handlers console_cohesion+
colcon test-result --test-result-base build/mini_nav_fastlivo --verbose
colcon test-result --test-result-base build/mini_nav_nodes --verbose
```

`tools/validate_mapping.py`、`validate_navigation.py`、`validate_frontend_fault.py` 是会启动仿真并驱动车辆的独立验收脚本，必须使用空闲的 `ROS_DOMAIN_ID=224`，并按建图、导航、故障的顺序执行。不会连接默认 219 域，也不访问手柄；脚本只停止自己启动的进程。
