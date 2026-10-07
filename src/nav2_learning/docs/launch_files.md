# Nav2 启动文件说明

本文说明 `nav2_learning/launch/` 下两个主要启动文件的作用和代码结构：

- `official_tb3_nav2.launch.py`：使用官方 TurtleBot3 仿真进行 SLAM 建图
- `localization_nav2.launch.py`：加载已经保存的地图，使用 AMCL 和 Nav2 进行自主导航

这两个文件本身主要是“启动封装”。真正的 Gazebo、SLAM、AMCL 和 Nav2 节点由 ROS 2 已安装的官方包提供。

## 一、整体关系

```text
nav2_learning 启动文件
        |
        | IncludeLaunchDescription
        v
nav2_bringup/launch/tb3_simulation_launch.py
        |
        +-- Gazebo Sim
        +-- TurtleBot3
        +-- robot_state_publisher
        +-- ros_gz_bridge
        +-- SLAM Toolbox 或 AMCL
        +-- Nav2
        +-- RViz2（可选）
```

官方启动文件通过 `slam` 参数决定工作模式：

```text
slam:=True   -> SLAM 建图模式
slam:=False  -> 地图定位导航模式
```

## 二、`official_tb3_nav2.launch.py`

文件路径：

```text
launch/official_tb3_nav2.launch.py
```

### 1. 导入模块

```python
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
```

这些模块分别用于：

- `os`：拼接文件路径
- `get_package_share_directory`：查找 ROS 2 包的安装目录
- `LaunchDescription`：描述一个启动文件包含哪些动作
- `IncludeLaunchDescription`：包含另一个启动文件
- `PythonLaunchDescriptionSource`：指定被包含的 Python 启动文件

### 2. 找到官方 Nav2 启动文件

```python
nav2_bringup = get_package_share_directory("nav2_bringup")
launch_file = os.path.join(
    nav2_bringup, "launch", "tb3_simulation_launch.py"
)
```

这里不会自己编写 Gazebo 或 Nav2 节点，而是找到系统中官方的：

```text
nav2_bringup/launch/tb3_simulation_launch.py
```

因此，这个文件是一个“官方启动文件的学习封装”。

### 3. 设置启动参数

```python
launch_arguments={
    "slam": "True",
    "headless": "True",
    "use_rviz": "False",
    "autostart": "True",
}.items(),
```

参数含义：

| 参数 | 值 | 作用 |
|---|---|---|
| `slam` | `True` | 启动 SLAM Toolbox 进行建图 |
| `headless` | `True` | Gazebo 无图形界面运行 |
| `use_rviz` | `False` | 不启动 RViz2 |
| `autostart` | `True` | 自动激活 Nav2 生命周期节点 |

### 4. 返回启动描述

```python
return LaunchDescription([
    IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_file),
        launch_arguments={...}.items(),
    )
])
```

这表示：启动本文件时，实际执行官方 `tb3_simulation_launch.py`，并把上面的参数传递给它。

### 5. 启动建图模式

```bash
export GZ_PARTITION=nav2_learning_demo
export ROS_DOMAIN_ID=42

source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash

ros2 launch nav2_learning official_tb3_nav2.launch.py
```

这个模式下，机器人使用激光雷达和里程计探索环境，SLAM Toolbox 发布地图。建图完成后，可以保存地图：

```bash
ros2 run nav2_map_server map_saver_cli \
  -f /home/a/ros2_ws/maps/tb3_learning
```

会生成：

```text
/home/a/ros2_ws/maps/tb3_learning.pgm
/home/a/ros2_ws/maps/tb3_learning.yaml
```

## 三、`localization_nav2.launch.py`

文件路径：

```text
launch/localization_nav2.launch.py
```

这个文件用于加载已经保存的地图，不再进行建图。

### 1. 声明启动参数

```python
DeclareLaunchArgument(
    "map",
    default_value="/home/a/ros2_ws/maps/tb3_learning.yaml",
)
```

`map` 表示地图 YAML 文件路径。YAML 文件会进一步引用对应的 PGM 地图图片。

```python
DeclareLaunchArgument("headless", default_value="False")
DeclareLaunchArgument("use_rviz", default_value="True")
```

这两个参数用于控制：

- 是否显示 Gazebo 图形界面
- 是否启动 RViz2

### 2. 获取启动参数

```python
map_file = LaunchConfiguration("map")
headless = LaunchConfiguration("headless")
use_rviz = LaunchConfiguration("use_rviz")
```

`LaunchConfiguration` 会在启动时读取命令行或默认值传入的参数。

### 3. 设置导航模式

```python
launch_arguments={
    "slam": "False",
    "map": map_file,
    "headless": headless,
    "use_rviz": use_rviz,
    "autostart": "True",
}.items(),
```

这里最重要的是：

```python
"slam": "False"
```

它告诉官方启动文件：

- 不启动 SLAM Toolbox
- 启动 Map Server
- 启动 AMCL
- 使用已有地图进行定位
- 启动 Nav2 规划和控制

导航阶段的数据流是：

```text
地图 + 激光雷达 + 里程计
              |
              v
             AMCL
              |
          map -> odom TF
              |
              v
             Nav2
              |
          /cmd_vel
              |
              v
           Gazebo
```

### 4. 启动已有地图导航

```bash
export GZ_PARTITION=nav2_learning_demo
export ROS_DOMAIN_ID=42

source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash

ros2 launch nav2_learning localization_nav2.launch.py
```

也可以临时指定另一张地图：

```bash
ros2 launch nav2_learning localization_nav2.launch.py \
  map:=/home/a/ros2_ws/maps/another_map.yaml
```

无图形界面启动：

```bash
ros2 launch nav2_learning localization_nav2.launch.py \
  headless:=True \
  use_rviz:=False
```

## 四、导航启动后的操作顺序

启动 `localization_nav2.launch.py` 后：

1. 在 RViz 中确认地图已经显示。
2. 点击 `2D Pose Estimate` 设置机器人初始位姿。
3. 等待 AMCL 建立 `map -> odom` 变换。
4. 点击 `Nav2 Goal` 设置目标点。
5. Nav2 会规划路径并通过 `/cmd_vel` 控制机器人运动。

如果没有设置初始位姿，通常会看到：

```text
AMCL cannot publish a pose or update the transform.
Please set the initial pose.
```

这不是启动失败，而是在提醒用户使用 `2D Pose Estimate` 初始化定位。

## 五、两个启动文件的区别

| 启动文件 | `slam` | 地图 | 主要用途 |
|---|---:|---|---|
| `official_tb3_nav2.launch.py` | `True` | 实时生成 | 建图 |
| `localization_nav2.launch.py` | `False` | 已保存 YAML | 定位与自主导航 |

核心切换只有一个：

```text
SLAM 建图：slam:=True
已有地图导航：slam:=False + map:=xxx.yaml
```

