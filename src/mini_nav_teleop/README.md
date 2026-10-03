# mini_nav USB PS5 手柄控制

复用用户的 `pydualforge` USB 驱动，将手柄输入交给现有 `velocity_guard_node`。

- 左摇杆上下：前进 / 后退，最大 2 m/s。
- 右摇杆左右：左转 / 右转，最大 2 rad/s。
- 按一次 L1：启用控制，短振动约 150 ms，LED 蓝色。
- 再按一次 L1：关闭控制并停车，短振动约 150 ms，LED 红色。
- L1 松开后保持当前开关状态；摇杆回中发送零速度。启动和重连默认为关闭、红灯。
- 摇杆死区 12%；左右摇杆未使用的轴不参与控制。
- 输入超过 0.25 秒未更新或断连时撤销运动许可；恢复后须重新按 L1 启用。
- 独立速度保护节点在桥接进程停止更新、仿真时钟停止/倒退、消息非法时发送零速。

当前入口只用于 FAST-LIVO2 仿真域 219、分区 `mini_nav_fastlivo_deploy`。
手动模式与自主导航分别启动：脚本在已有速度/原始命令/运动许可发布者时拒绝启动，
桥接节点也会检查发布者冲突并撤销运动许可。

## 运行

连接 PS5 USB 数据线，直接启动完整仿真与控制面板：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash
source /home/a/ros2_ws/install_fastlivo/setup.bash
export ROS_DOMAIN_ID=219
export GZ_PARTITION=mini_nav_fastlivo_deploy
ros2 launch fastlivo_sim bringup.launch.py map_dir:=/home/a/ros2_ws/maps/fastlivo2
```

新总入口默认 `teleop:=true`：预先启动手柄节点和速度保护，初始控制关闭、红灯。
RViz 默认显示 **PS5 手柄控制** 面板：

- “启用控制”：需要手柄在线、输入新鲜、仿真时钟正常且无速度发布者冲突；启用时蓝灯、短振动。
- “关闭控制 / 停车”：撤销运动许可、发送零速度，红灯、短振动；输入故障时也可请求关闭。
- 两个限速输入框：范围 0–2 m/s、0–2 rad/s；点击“应用速度上限”原子更新两项设置。
- “实际限速”：显示节点确认的配置，区别于输入框中尚未应用的值。
- L1 与面板共用开关；断连、超时、时钟异常或发布者冲突后不会自动恢复运动。

修改 launch 后须重新启动才能自动加入新节点。旧仿真没有手柄节点时，可单独运行原入口：

```bash
bash /home/a/ros2_ws/src/fastlivo2_deploy/scripts/teleop_ps5.sh
```

等待 `DualSense connected` 后，按一次 L1，再操作两个摇杆；再次按 L1 关闭控制。
停止手柄控制终端用 Ctrl+C。
完整入口已启动手柄节点时不再运行 `teleop_ps5.sh`，重复启动会被拒绝。
想只运行仿真可传 `teleop:=false`，再选择需要的控制入口。
不要同时运行 `probe --drive`、键盘遥控或完整自主导航。

数据链：

```text
PS5 USB -> user's DualForge parser -> ps5_teleop
   -> /mini_nav/cmd_vel_raw + /mini_nav/task_active
   -> velocity_guard -> /cmd_vel (TwistStamped) -> Gazebo Waffle
```

手动速度保护的绝对上限始终为 2 m/s、2 rad/s；桥接节点可在此范围内动态调节限速。
`velocity_guard_node` 新增 `max_linear_speed`、`max_angular_speed` 参数，默认仍为
0.15 m/s、0.6 rad/s，原自主导航启动配置不受影响。参数必须为有限正数且不超过 2。
需要降低手动上限可传入参数：

```bash
bash /home/a/ros2_ws/src/fastlivo2_deploy/scripts/teleop_ps5.sh \
  linear_limit:=1.0 angular_limit:=1.0
```

## 驱动来源与 Linux 适配

`vendor/dualforge` 是用户本地项目的源码快照。来源路径、提交及各原文件 SHA-256
见 `vendor/provenance.json`，其来源包含用户尚未提交的改动。原盘上的项目未修改。

兼容副本仅调整：Windows DLL 搜索目录调用加平台判断；只接收 64 字节、Report ID 0x01 的 USB 输入；
设备断开时先等待读取线程结束再关闭 HID；枚举时筛选 USB 或 bus 未知的主接口。
原用户注释保留。此入口不支持蓝牙。

Python 依赖为 `hid`（pyhidapi 的 `Device` API），不是接口不同的 `hidapi` Python 包。
固定版本和 wheel SHA-256 见 `requirements.lock`。
依据：[pyhidapi 官方说明](https://github.com/apmorton/pyhidapi)、[hid 1.0.9 发布文件](https://pypi.org/project/hid/1.0.9/)。
使用主机已经安装的 Linux hidapi 动态库。

`hid` 1.0.9 的原始绑定和 MIT 许可证现在一起放在 `vendor/hid`，随包安装到节点目录，
直接 `ros2 launch` 可导入，不依赖终端手工设置 PYTHONPATH。原安装目录
`install_fastlivo_deps/python` 和锁定依赖继续保留；没有修改系统 Python。

## 重建和测试

```bash
cd /home/a/ros2_ws
python3 -m pip install --no-deps --require-hashes \
  --target install_fastlivo_deps/python -r src/mini_nav_teleop/requirements.lock
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select mini_nav_nodes mini_nav_teleop --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select mini_nav_nodes mini_nav_teleop --event-handlers console_cohesion+
colcon test-result --test-result-base build/mini_nav_nodes --verbose
colcon test-result --test-result-base build/mini_nav_teleop --verbose
```

依赖已构建的 `mini_nav_nodes`；缺失时先构建 `mini_nav_core mini_nav_nodes`。
测试覆盖双摇杆映射、死区、L1 上升沿切换、保持/释放、非法输入、超时、断连、重连和外部门控。
2026-10-03 用户实测确认 USB 控制、L1 开关、蓝/红灯和振动正常。
详细记录见 [验证报告](../mini_nav/logs/26-10-3/ps5_dualforge_gazebo_control.md)。
新增面板的接口和整链路验证见 [RViz PS5 面板报告](../mini_nav/logs/26-10-3/ps5_rviz_control_panel.md)。
