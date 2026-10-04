# AMCL 定位核心

原 `localization/` 下的六个实现文件集中到这里：地图距离场、差分运动模型、Beam / Likelihood Field 激光模型、粒子分箱与粒子滤波。

公开头文件仍在 `include/mini_nav_core/localization/`，命名空间和接口保持不变。`mini_nav_core` 的 CMake 显式编译本目录；ROS 节点仍是 `mini_nav_nodes/src/amcl_node.cpp`。
