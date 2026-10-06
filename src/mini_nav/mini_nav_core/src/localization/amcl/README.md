# AMCL 定位核心

原 `localization/` 下的六个实现文件集中到这里：地图距离场、差分运动模型、Beam / Likelihood Field 激光模型、粒子分箱与粒子滤波。

公开头文件在 `include/mini_nav_core/localization/amcl/`。普通位姿与变换复用 `nav_types/`；AMCL 命名空间保留类型别名。`mini_nav_core` 的 CMake 显式编译本目录；ROS 节点是 `mini_nav_nodes/src/amcl/amcl_node.cpp`。
