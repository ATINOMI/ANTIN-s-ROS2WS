# nav2_learning 学习包

`nav2_learning` 是一个用于学习 ROS 2 Jazzy 和 Nav2 的示例包。它将机器人模型、Gazebo 仿真、传感器桥接、SLAM 建图、地图保存、AMCL 定位和 Nav2 自主导航组织在同一个包中。

## 1. 功能概览

```text
机器人模型
    ↓
Gazebo Sim
    ↓
ros_gz_bridge
    ↓
/scan、/odom、/tf、/cmd_vel
    ↓
SLAM 或 AMCL
    ↓
Nav2 规划与控制
```

当前包包含两套模型：

- `urdf/learning_robot.urdf.xacro`：我们自己编写的简化机器人模型，用于学习 URDF、Xacro 和 Gazebo 插件。
- `urdf/official_turtlebot3_waffle.sdf.xacro`：移植到本包中的官方 TurtleBot3 Waffle 模型，用于运行稳定的 Nav2 示例。

当前完整建图和导航流程使用移植后的官方 TurtleBot3 模型。

## 2. 环境要求

- Ubuntu
- ROS 2 Jazzy
- Gazebo Sim
- Nav2
- `ros_gz_sim`
- `ros_gz_bridge`
- `teleop_twist_keyboard`（建图遥控时使用）

加载 ROS 2 和工作空间环境：

```bash
source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash
```

每个新终端都需要重新执行 `source`。也可以加入 `~/.bashrc`：

```bash
echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
echo 'source /home/a/ros2_ws/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

## 3. 编译学习包

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_learning
source install/setup.bash
```

检查包是否可以被 ROS 2 找到：

```bash
ros2 pkg list | grep nav2_learning
```

## 4. 目录结构

```text
nav2_learning/
├── CMakeLists.txt
├── package.xml
├── src/
│   └── hello_nav2.cpp
├── urdf/
│   ├── learning_robot.urdf.xacro
│   ├── official_turtlebot3_waffle.sdf.xacro
│   └── official_turtlebot3_waffle.urdf
├── models/
│   └── turtlebot3_model/
│       └── meshes/
├── config/
│   ├── bridge.yaml
│   └── nav2_params.yaml
├── rviz/
│   └── nav2_default_view.rviz
├── launch/
│   ├── display_robot.launch.py
│   ├── sim.launch.py
│   ├── official_tb3_nav2.launch.py
│   └── localization_nav2.launch.py
└── docs/
    ├── README.md
    └── launch_files.md
```
### 文件和目录作用说明

| 文件或目录 | 作用 |
|---|---|
| `CMakeLists.txt` | 编译 C++ 节点，并安装模型、配置、启动文件和 RViz 文件。 |
| `package.xml` | ROS 2 包清单，记录依赖和包信息。 |
| `.gitignore` | 指定 Git 忽略的临时文件。 |
| `src/hello_nav2.cpp` | 最小 `rclcpp` 节点，用于验证 C++ 包。 |
| `urdf/learning_robot.urdf.xacro` | 自己编写的简化机器人模型。 |
| `urdf/official_turtlebot3_waffle.sdf.xacro` | 移植后的官方 TurtleBot3 Gazebo 模型。 |
| `urdf/official_turtlebot3_waffle.urdf` | 移植后的官方 TurtleBot3 URDF 和 TF 参考模型。 |
| `models/turtlebot3_model/` | 官方 TurtleBot3 的网格和 Gazebo 模型资源。 |
| `config/bridge.yaml` | 自定义模型的 Gazebo/ROS 2 话题桥接配置。 |
| `config/nav2_params.yaml` | AMCL、Costmap、Planner、Controller 等 Nav2 参数。 |
| `rviz/nav2_default_view.rviz` | RViz 默认显示配置。 |
| `launch/display_robot.launch.py` | 只启动 `robot_state_publisher` 检查自定义 TF。 |
| `launch/sim.launch.py` | 启动自定义机器人和 Gazebo 仿真。 |
| `launch/official_tb3_nav2.launch.py` | 使用本地官方模型启动 SLAM 建图。 |
| `launch/localization_nav2.launch.py` | 使用本地模型和保存地图启动 AMCL 与 Nav2。 |
| `docs/README.md` | 学习包总览和使用说明。 |
| `docs/launch_files.md` | 启动文件代码和工作原理说明。 |

三个核心配置的关系是：

```text
机器人模型 → Gazebo/ROS 2 桥接 → Nav2 参数 → 启动文件 → Gazebo + AMCL/SLAM + Nav2 + RViz
```

其中，`learning_robot.urdf.xacro` 是自定义学习模型；完整导航流程使用移植后的 `official_turtlebot3_waffle.sdf.xacro`。`nav2_params.yaml` 和 `nav2_default_view.rviz` 已复制到本包，并由两个 Nav2 封装启动文件使用。

## 5. 启动自己的简化模型

这个启动文件只用于检查我们自己编写的 Xacro 和 TF：

```bash
ros2 launch nav2_learning display_robot.launch.py
```

它只启动 `robot_state_publisher`，不会启动 Gazebo 或 Nav2。

启动自己编写的 Gazebo 仿真：

```bash
ros2 launch nav2_learning sim.launch.py
```

这个流程用于学习：

- Gazebo 模型生成
- 差速驱动插件
- 激光雷达
- `ros_gz_bridge`
- `/cmd_vel`、`/odom` 和 `/scan`

## 6. SLAM 建图

使用移植后的官方 TurtleBot3 模型启动 SLAM：

```bash
export GZ_PARTITION=nav2_learning_demo
export ROS_DOMAIN_ID=42

source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash

ros2 launch nav2_learning official_tb3_nav2.launch.py
```

该启动文件默认：

- `slam:=True`
- `headless:=True`
- `use_rviz:=False`
- `autostart:=True`

如果需要 Gazebo 和 RViz 界面，可以直接使用官方入口：

```bash
ros2 launch nav2_bringup tb3_simulation_launch.py \
  slam:=True \
  headless:=False \
  use_rviz:=True
```

另开终端启动键盘遥控：

```bash
export GZ_PARTITION=nav2_learning_demo
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

常用按键：

```text
i：前进
,：后退
j：左转
l：右转
k：停止
```

## 7. 保存地图

建图完成后执行：

```bash
export ROS_DOMAIN_ID=42
source /opt/ros/jazzy/setup.bash

ros2 run nav2_map_server map_saver_cli \
  -f /home/a/ros2_ws/maps/tb3_learning
```

生成的地图文件是：

```text
/home/a/ros2_ws/maps/tb3_learning.pgm
/home/a/ros2_ws/maps/tb3_learning.yaml
```

## 8. 使用已有地图导航

启动 `localization_nav2.launch.py`：

```bash
export GZ_PARTITION=nav2_learning_demo
export ROS_DOMAIN_ID=42

source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash

ros2 launch nav2_learning localization_nav2.launch.py \
  headless:=False \
  use_rviz:=True
```

该启动文件默认加载：

```text
/home/a/ros2_ws/maps/tb3_learning.yaml
```

也可以指定其他地图：

```bash
ros2 launch nav2_learning localization_nav2.launch.py \
  map:=/home/a/ros2_ws/maps/another_map.yaml
```

启动后在 RViz 中依次操作：

1. 使用 `2D Pose Estimate` 设置机器人初始位置和朝向。
2. 等待 AMCL 建立 `map -> odom` 变换。
3. 使用 `Nav2 Goal` 设置目标点。
4. 观察 Nav2 规划路径并控制机器人运动。

## 9. 常见问题

### 找不到 `nav2_learning`

重新编译并加载工作空间：

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_learning
source install/setup.bash
```

### `file 'None' was not found`

通常是启动命令换行或参数写错。正确格式是：

```bash
ros2 launch nav2_learning localization_nav2.launch.py
```

包名和启动文件名之间不要加 `/`。

### AMCL 提示需要初始位姿

这是正常提示。在 RViz 中点击 `2D Pose Estimate` 设置初始位姿即可。

### Gazebo 找不到网格文件

确认使用本包安装后的环境：

```bash
source /home/a/ros2_ws/install/setup.bash
```

然后重新编译：

```bash
colcon build --packages-select nav2_learning
```

## 10. 进一步学习顺序

建议按下面顺序继续学习：

1. 理解 Xacro、Link、Joint 和 TF。
2. 查看 `config/bridge.yaml` 中的 Gazebo/ROS 话题桥接。
3. 阅读 `config/nav2_params.yaml` 中的 Costmap、Planner 和 Controller 参数。
4. 修改 RViz 配置并观察 `/scan`、`/odom`、`/tf`。
5. 调整 AMCL 参数并观察定位效果。
6. 修改 Nav2 Planner 和 Controller 参数。
7. 最后再编写自己的导航启动流程，逐步减少对官方启动文件的依赖。

