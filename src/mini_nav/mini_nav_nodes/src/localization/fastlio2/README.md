# FAST-LIO2 的 ROS 节点与接入层

- `fast_lio/`：上游 `fast_lio` 包。`src/laserMapping.cpp` 包含点面观测、先验地图模式与 ROS 发布；`IMU_Processing.hpp` 和 `preprocess.*` 处理 ROS IMU / 点云消息。
- `icp_relocalization/`：上游初值邻域 ICP 包，声明 Apache-2.0 许可。
- `adapter/`：仿真输入、IMU 到车体转换、定位质量 / TF 接入、`/initialpose` 会话及后端子进程管理。

两个上游 ROS 包用独立的 colcon 构建显式发现；不会加入 `mini_nav_nodes` 的基础目标。Python 接入层由 `scurm_sim` 安装，保持原可执行文件、节点、话题和 launch 名称。

计算内核移至 `mini_nav_core/src/localization/fastlio2/`，仿真配置与运行工具移至 `mini_nav_fastlio/`。原默认 AMCL 入口保持可用。详细位置见 [定位后端目录说明](../../../../docs/localization_backends.md)。
