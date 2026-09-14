# nav2_stvl_demo

这是一个与 `nav2_learning` 隔离的 Nav2 STVL 学习案例。

STVL 的全称是 `Spatio-Temporal Voxel Layer`，它将传感器点云放入带有高度和时间衰减信息的体素网格中，用于构建局部代价地图。

## 与 nav2_learning 的隔离方式

- 独立 ROS 2 包：`nav2_stvl_demo`
- 独立启动文件：`stvl_nav2.launch.py`
- 独立 Nav2 参数文件
- 独立机器人模型和网格资源
- 建议使用独立 `ROS_DOMAIN_ID`
- 建议使用独立 `GZ_PARTITION`

它不会修改或替换 `nav2_learning` 的模型、参数和启动文件。

## 目录结构

```text
nav2_stvl_demo/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── nav2_params.yaml
├── launch/
│   └── stvl_nav2.launch.py
├── models/
│   └── turtlebot3_model/
├── urdf/
│   └── official_turtlebot3_waffle.sdf.xacro
└── src/
    └── scan_to_cloud.cpp
```

## 数据流程

```text
Gazebo /scan
      ↓
scan_to_cloud
      ↓ /stvl/points
SpatioTemporalVoxelLayer
      ↓
local_costmap
      ↓
Nav2 Controller
```

`scan_to_cloud` 是一个学习用适配节点，将 `sensor_msgs/msg/LaserScan` 转换为 `sensor_msgs/msg/PointCloud2`，发布到 `/stvl/points`。这样可以在当前 TurtleBot3 仿真环境中验证 STVL 的插件加载和点云输入流程。

注意：当前适配器由二维激光扫描生成点云，点云的 `z` 值为 0。它主要用于学习 STVL 的配置和数据接口，不等同于真实 3D 激光雷达或深度相机点云。接入真实 3D 传感器时，只需让 STVL 订阅真实的 `PointCloud2` 话题。

## 编译

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select nav2_stvl_demo
source install/setup.bash
```

## 启动

推荐使用独立的 ROS 域和 Gazebo 分区：

```bash
export ROS_DOMAIN_ID=48
export GZ_PARTITION=nav2_stvl_demo

source /opt/ros/jazzy/setup.bash
source /home/a/ros2_ws/install/setup.bash

ros2 launch nav2_stvl_demo stvl_nav2.launch.py \
  headless:=False \
  use_rviz:=True
```

启动后，在 RViz 中设置初始位姿，然后可以发送 Nav2 目标点。

## 关键 STVL 参数

配置文件：

```text
config/nav2_params.yaml
```

局部代价地图使用：

```yaml
plugins: ["stvl_layer", "inflation_layer"]
```

STVL 插件：

```yaml
stvl_layer:
  plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
  voxel_decay: 15.0
  decay_model: 0
  voxel_size: 0.05
  publish_voxel_map: true
```

点云输入：

```yaml
observation_sources: pointcloud
pointcloud:
  data_type: PointCloud2
  topic: /stvl/points
  marking: true
  clearing: true
```

## 与普通 VoxelLayer 的区别

普通 `VoxelLayer` 主要维护固定的三维体素网格；STVL 额外提供体素随时间衰减的能力，适合动态障碍物、3D 激光雷达和深度相机等场景。

当前案例只修改 `nav2_stvl_demo/config/nav2_params.yaml` 中的局部代价地图插件，不会影响 `nav2_learning/config/nav2_params.yaml`。

