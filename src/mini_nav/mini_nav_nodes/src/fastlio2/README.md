# FAST-LIO2 的 ROS 节点与接入层

- `fast_lio/`：独立 ROS 包；`src/main.cpp` 为入口，`src/node/` 按节点调度、参数、传感器输入及输出分文件。
- `icp_relocalization/`：上游初值邻域 ICP 包，本轮只迁移目录，未拆分其内部算法。
- `adapter/`：仿真输入、IMU 到车体转换、定位质量/TF、初值会话及后端进程管理。

FAST-LIO2 项目头文件集中于 `mini_nav_nodes/include/mini_nav_nodes/fastlio2/`。私有方法按职责实现于不同 cpp，不为每个 cpp 增设转发头文件。节点链接 core 导出的可选计算库，ROS 解码、同步、参数和发布留在本层。

两个独立 ROS 包通过独立 colcon 构建发现，不加入 mini_nav_nodes 的基础目标。Python adapter 由 scurm_sim 安装，ROS 包名、可执行名、话题和 launch 保持兼容。运行说明见 [定位后端目录说明](../../../docs/localization_backends.md)。
