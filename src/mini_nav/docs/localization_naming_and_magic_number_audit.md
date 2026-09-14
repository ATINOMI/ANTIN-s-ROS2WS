# localization 命名与魔法数字整改报告

整改日期：2026-09-12

范围：`mini_nav_core/include/mini_nav_core/localization/`、`mini_nav_core/src/localization/`，以及受影响的 `mini_nav_nodes` 调用点和项目文档。

## 结论

本轮已完成两类阻塞问题修复，并将定位核心中最容易误读的名称、单位和数值保护集中整理。核心仍保持 ROS 无关；ROS 参数名（包括 `pf_z`、`alpha1` 到 `alpha5`）保持不变，进入核心后使用语义化字段。旧入口通过别名或转发保留，现有调用者可以渐进迁移。

## 已完成的修复

### 编译与接口

- 调整 `types.hpp` 中 `Covariance3` 与 `PoseEstimate` 的定义顺序，修复值成员使用未完成类型的问题。
- 修复 `beam_model.cpp` 多余命名空间闭合，BeamModel 实现回到 `mini_nav_core::localization`。
- 新增 `PoseCovariance` 语义别名、`DiagonalVariances()`，保留 `Covariance3`、`Diagonal()` 和 `Values()` 兼容入口。
- 新增 `pose_utils.hpp`，Beam 和 Likelihood Field 共用 `ComposePose2D()`。

### 命名与坐标语义

- `KdTree` 的 canonical 名称改为 `PoseBinIndex`；内部键和容器改为 `PoseBinKey`、`bin_id_by_key_`、`bin_id_by_particle_index_`、`particle_indices_by_bin_id_`。`KdTree` 和 `Build()` 保留为兼容别名/转发。
- `MapCell` 的 canonical 名称改为 `GridCell`；新增 `IsKnownFree()`、`IsKnownOccupied()`、`IsUnknown()`，旧的 `IsFree()` 和 `IsOccupied()` 保留转发。
- 新增 `TryGetObstacleDistanceAtWorld()`，越界返回值与有效距离可以区分；旧的 `GetObstacleDistanceAtWorld()` 仍返回最大距离哨兵。
- `LaserModel` 增加 `ApplyMeasurementLikelihood()`；旧 `UpdateWeights()` 作为兼容入口。激光外参统一命名为 `base_to_laser_pose`，表达 `T_base_laser`。
- 粒子滤波选项中的 `pf_z` 改为核心字段 `kld_normal_quantile`；节点仍读取 ROS 参数 `pf_z`。`PoseEstimate::weight` 的注释明确它是估计使用的粒子权重总和。
- 删除未实现的 `InitializeWithSamples()` 声明；`alpha5` 注释明确为 Nav2 兼容字段，当前差速模型不使用它。

### 魔法数字与输入边界

新增 `localization_constants.hpp`，集中定义 π、平方根二、默认运动噪声、默认随机种子、默认分箱尺寸、最大障碍距离、射线步长、角度阈值、概率/指数保护边界、协方差容差和哈希组合常量。地图邻居改为带 `dx`、`dy`、`distance_multiplier` 字段的结构，避免用 `1/0` 表示步长类型；Acklam 正态分位数系数集中在具名结构中。

`PoseBinIndex` 现在会拒绝 NaN、无穷大和超出 `int` 分箱范围的位姿，避免浮点转整数的未定义行为。Beam 构造函数现在校验四个混合权重的有限性、非负性和正总和，同时继续校验标准差、指数参数和最大束数。

## 验证证据

- `g++ -std=c++17 -Wall -Wextra -Wpedantic -fsyntax-only` 对六个 localization `.cpp` 全部通过。
- `source /opt/ros/jazzy/setup.bash && colcon build --packages-select mini_nav_core mini_nav_nodes mini_nav_bringup --event-handlers console_cohesion+` 通过，三包均完成构建。
- `colcon test --packages-select mini_nav_core mini_nav_nodes --event-handlers console_cohesion+` 通过，所选包共 26 个测试全部通过；`colcon test-result --verbose` 的整个工作区汇总为 **35 tests, 0 errors, 0 failures, 0 skipped**。
- 新增 `test_localization_naming.cpp`，覆盖位姿分箱边界、非有限位姿、地图三态/越界距离和 Beam 非法混合权重。
- `ros2 launch mini_nav_bringup mini_localization_astar.launch.py --show-args` 通过。
- `git diff --check` 通过；未修改工作区中与本任务无关的 dirty 文件，也未纳入 build/install/log 或既有 `*.orig` 文件。

## 尚未实现但已明确的边界

- `recovery_alpha_fast/slow` 仍只是配置字段，`fast_mean_weight_`、`slow_mean_weight_` 尚未驱动随机恢复；后续实现应单独补算法和测试。
- `Estimate()` 仍对全部粒子做加权均值，尚未按最高权重位姿分箱簇估计；这属于算法行为变更，应另立任务验证多峰场景。
- 节点中部分 Nav2 兼容参数（beam skip、保存位姿等）仍只为配置文件兼容而声明，不能视为已实现功能。
