# SCURM FAST-LIO2 计算内核

来源为 `PolarisXQ/SCURM_SentryNavigation@46e6425c692ec98f8e65446fb6fdd360f44ef8e5`。2026-10-06 拆分前版本已保存为 `f2218c9`。

- `fastlio_estimator.cpp`：实例状态、点面观测、迭代滤波及地图更新。
- `imu_processing.hpp`：普通 IMU 数据的初始化、预测与去畸变。
- `pointcloud_preprocess.cpp`：已解码点云与 Livox 普通数据的预处理。
- `IKFoM_toolkit/`、`use-ikfom.hpp`：流形滤波、状态和过程模型。
- `ikd-Tree/`：增量点云索引；`so3_math.h`、`Exp_mat.h`：旋转工具。

公开接口在 `include/mini_nav_core/localization/fastlio2/`，以上文件是计算库私有实现。`MINI_NAV_BUILD_FASTLIO2=ON` 构建 `mini_nav_fastlio2_core`，依赖 Eigen/PCL/OpenMP，无 ROS 依赖；OFF 时不将三维依赖带入默认二维库。日志仅按配置启用，不依赖 nodes 的源码目录。

原注释、GPL-2.0 LICENSE 与文件级许可声明保留。实例观测回调及点云/日志保护的改动见 [定位后端说明](../../../../docs/localization_backends.md)和 [实施报告](../../../../logs/26-10-6/core_node_module_partition_implementation.md)。
