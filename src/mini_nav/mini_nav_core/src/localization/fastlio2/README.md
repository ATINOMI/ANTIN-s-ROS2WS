# SCURM FAST-LIO2 计算内核

来源为 `PolarisXQ/SCURM_SentryNavigation@46e6425c692ec98f8e65446fb6fdd360f44ef8e5`，迁移前的已验证源码保存在工作区 Git 提交 `94b4189`。

- `IKFoM_toolkit/`：流形上的误差状态迭代滤波。
- `use-ikfom.hpp`：状态、输入、噪声与过程模型。
- `ikd-Tree/`：增量三维点云索引。
- `so3_math.h`、`Exp_mat.h`：旋转数学工具。

这些上游文件保留原代码、注释与许可。它们不引用 ROS 消息；ikd-Tree 依赖 PCL，滤波内核依赖 Eigen。FAST-LIO2 包显式引用本目录编译，并未将三维依赖加入基础 `mini_nav_core` 库。

完整的 IMU/点云处理、点面观测、ICP 及 ROS 节点见 `mini_nav_nodes/src/localization/fastlio2/`。构建、启动、版本与边界说明见 [定位后端目录说明](../../../../docs/localization_backends.md)。`LICENSE` 保留 FAST_LIO 的 GPL-2.0 许可，其他文件中的上游声明同样保留。
