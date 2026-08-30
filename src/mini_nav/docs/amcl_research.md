# ROS 2 Jazzy/Nav2 AMCL 学习资料

最后更新：2026-08-27<br>
适用环境：ROS 2 Jazzy、Nav2 `nav2_amcl`，本机安装包版本为 `1.3.12-1noble.20260615.153115`。

本文的目标不是把 AMCL 当成一个“黑盒定位节点”来使用，而是把它拆成可以逐步复现的算法和 ROS 2 接口。资料优先采用 Nav2 Jazzy 分支源码、Nav2 Jazzy API 文档、ROS REP-105 和原始论文；Nav2 网站上的参数页目前是 Rolling 页面，因此涉及版本差异时以 Jazzy 分支源码为准。

## 1. AMCL 到底做什么

AMCL 是 **Adaptive Monte Carlo Localization**，即自适应蒙特卡洛定位。它在一张已知的二维地图中，利用二维激光扫描和机器人运动信息，估计机器人在地图坐标系中的二维位姿 `(x, y, yaw)`。Nav2 Jazzy 的包说明明确把它描述为：用粒子滤波跟踪机器人相对于已知地图的位姿，并采用 Adaptive/KLD-sampling Monte Carlo Localization 方法。[Nav2 Jazzy `nav2_amcl` 文档](https://docs.ros.org/en/jazzy/p/nav2_amcl/)

AMCL 解决的是“我在已知地图中的什么位置”，不是以下问题：

- 它不负责创建地图；地图通常由 `map_server` 或 SLAM 系统提供。
- 它不负责从起点到目标点搜索路径；这属于规划器，例如本项目自己的 A*。
- 它不负责让机器人沿路径运动；这属于路径跟踪器/控制器。
- 它不直接发布 `map -> base_footprint` 作为唯一 TF，而是结合里程计发布 `map -> odom`，让已有的 `odom -> base_*` 继续发挥作用。[REP-105 坐标系规范](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)

从概率角度看，机器人真实位姿是未知状态，AMCL 用一组带权粒子近似“机器人可能在哪里”的概率分布。Fox 等人在 1999 年的原始 MCL 论文中，将 MCL 描述为基于采样的 Markov 定位方法：用样本集合表示可能的位姿分布，并根据运动和观测不断更新。[AAAI-99 原始论文](https://aaai.org/papers/050-aaai99-050-monte-carlo-localization-efficient-position-estimation-for-mobile-robots/)

## 2. 输入、输出和生命周期

### 2.1 主要输入

| 数据 | 默认接口 | 作用 | Jazzy 实现依据 |
|---|---|---|---|
| 静态地图 | `map`，`nav_msgs/msg/OccupancyGrid` | 将地图中的障碍和自由空间用于初始化、激光匹配 | Jazzy 源码以可靠、Transient Local QoS 订阅地图：[amcl_node.cpp](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1457-L1465) |
| 激光扫描 | `scan`，`sensor_msgs/msg/LaserScan` | 用当前激光观测给每个粒子打分 | Jazzy 源码使用 sensor-data QoS 和 TF MessageFilter：[amcl_node.cpp](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1424-L1442) |
| 里程计 TF | `odom -> base_frame_id` | 根据机器人运动预测粒子如何移动 | Jazzy 源码在激光时间戳处获取 odom 位姿：[amcl_node.cpp](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L612-L620) |
| 激光安装 TF | `base_frame_id -> laser_frame` | 将激光观测转换到机器人基座和地图模型中 | Jazzy 源码首次收到扫描时查找激光到基座的 TF：[amcl_node.cpp](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L692-L711) |
| 初始位姿 | `initialpose`，`geometry_msgs/msg/PoseWithCovarianceStamped` | 告诉 AMCL 初始位姿和不确定度 | Jazzy 源码要求消息 frame 与 `global_frame_id` 一致：[amcl_node.cpp](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L488-L523) |

这里的 `map`、`scan`、`initialpose` 是 AMCL 的默认相对名称。在根命名空间下通常表现为 `/map`、`/scan`、`/initialpose`；如果使用 namespace，实际绝对名称会随命名空间变化。

AMCL 的激光回调并不是“收到一帧扫描就无条件计算”。Jazzy 源码先检查节点是否 active、地图是否收到，再查找与扫描时间对应的机器人 odom 位姿；任意条件不满足，当前扫描都可能被跳过。[Jazzy `laserReceived()` 实现](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L585-L620)

### 2.2 主要输出

| 输出 | 类型/TF | 作用 |
|---|---|---|
| `amcl_pose` | `geometry_msgs/msg/PoseWithCovarianceStamped` | 当前估计位姿和协方差 |
| `particle_cloud` | `nav2_msgs/msg/ParticleCloud` | 每个粒子的位置和权重，便于 RViz 查看收敛情况 |
| `map -> odom` | 动态 TF | 把全局地图定位结果与连续但会漂移的里程计坐标接起来 |
| `reinitialize_global_localization` | `std_srvs/srv/Empty` | 在地图自由空间中重新进行全局均匀初始化 |
| `set_initial_pose` | `nav2_msgs/srv/SetInitialPose` | 通过服务设置初始位姿 |
| `request_nomotion_update` | `std_srvs/srv/Empty` | 即使机器人没有移动，也请求一次观测更新 |

Jazzy 源码明确创建了 `particle_cloud`、`amcl_pose`、`initialpose` 和地图订阅，并注册上述服务。[AMCL 发布、订阅与服务初始化](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1445-L1479)

AMCL 是一个 lifecycle node。它在 configure 阶段初始化粒子滤波器、激光模型、TF 和订阅者，在 activate 阶段激活生命周期发布者并开始处理传感器回调；Nav2 的官方 `localization_launch.py` 用 lifecycle manager 管理 `map_server` 和 `amcl`。[AMCL 生命周期实现](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L244-L282)；[Jazzy localization launch](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/localization_launch.py#L106-L156)

## 3. 粒子滤波的核心流程

可以把 AMCL 的一次更新理解成下面这条循环：

```text
地图 + 初始位姿
       ↓
生成带权粒子
       ↓
里程计运动模型：预测每个粒子的新位置
       ↓
激光观测模型：比较地图预测与真实 LaserScan
       ↓
归一化权重
       ↓
重采样 / KLD 自适应调整粒子数量
       ↓
计算最可信位姿和协方差
       ↓
发布 amcl_pose、particle_cloud、map -> odom
```

这正对应 Bayes filter 的预测—校正结构。用概念公式写为：

\[
bel(x_t) = \eta\,p(z_t\mid x_t,m)
       \int p(x_t\mid u_t,x_{t-1}),bel(x_{t-1})\,dx_{t-1}
\]

其中：

- `x_t` 是机器人当前二维位姿；
- `u_t` 是从里程计得到的运动增量；
- `z_t` 是当前激光扫描；
- `m` 是已知地图；
- `p(x_t | u_t, x_{t-1})` 是运动模型；
- `p(z_t | x_t, m)` 是激光传感器模型；
- `η` 是归一化因子。

上式是对 MCL/Bayes filter 的学习化表达，不是把 Nav2 源码中的浮点实现逐字翻译。其理论出处是 Fox 等人的 MCL 论文；其在 Jazzy 中的具体执行顺序可以在激光回调里看到。[Fox 等人 1999 MCL 论文](https://aaai.org/papers/050-aaai99-050-monte-carlo-localization-efficient-position-estimation-for-mobile-robots/)；[Jazzy 激光回调中的运动更新、传感器更新和重采样](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L622-L676)

### 3.1 初始化：先告诉它大概在哪里

当前项目的 `amcl_waffle.yaml` 设置了 `set_initial_pose: false`，因此 AMCL 不会默认把机器人硬编码到 `(0, 0, 0)`，而是等待 RViz 的 `SetInitialPose` 发送 `/initialpose`。[本项目 AMCL 参数](../mini_nav_bringup/config/amcl_waffle.yaml)

收到初始位姿后，Jazzy 源码会：

1. 检查消息格式和 frame；
2. 将初始位姿与必要的 odom 变化对齐；
3. 从协方差中取出 `x`、`y` 和 `yaw` 的不确定度；
4. 以该均值和协方差初始化粒子滤波器。[Jazzy 初始位姿处理](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L488-L583)

如果调用 `reinitialize_global_localization`，AMCL 会改为在地图自由空间中均匀选择位置，并在 `[-π, π]` 内随机选择朝向；Jazzy 源码的新版均匀采样路径直接从自由空间索引中抽样。[Jazzy 全局初始化实现](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L418-L467)

因此要区分两种启动方式：

- **位置跟踪初始化**：用户提供一个合理的近似初始位姿，粒子集中在这个位姿附近，收敛较快。
- **全局定位初始化**：不知道机器人在哪里，粒子分布在整张地图自由空间中，需要更多观测和计算，也可能存在对称环境歧义。

如果 AMCL 尚未知道初始位姿，Jazzy 源码中的 `sendMapToOdomTransform()` 会直接返回，因此只启动 AMCL 而不设置初始位姿时，看不到有效 `map -> odom` 是正常现象。[Jazzy `sendMapToOdomTransform()`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L953-L960)

### 3.2 预测：用里程计移动每一个粒子

对于差速机器人，Nav2 使用 `DifferentialMotionModel`。它把一次 odom 增量拆成：

```text
delta_rot1 + delta_trans + delta_rot2
```

然后对每个粒子加入与运动相关的随机噪声，再把增量应用到粒子自己的朝向上。Jazzy 的实现直接按这一分解计算旋转和位移噪声，并更新每个粒子的 `x/y/yaw`。[Jazzy 差速运动模型](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/motion_model/differential_motion_model.cpp#L38-L104)

五个 `alpha` 参数描述不同运动来源的噪声：`alpha1/alpha2` 影响旋转噪声，`alpha3/alpha4` 影响平移噪声，`alpha5` 只用于全向模型。官方参数说明对这些含义有逐项定义。[Nav2 AMCL 参数说明](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/)

直观理解是：里程计告诉 AMCL“机器人应该向前走了多少、转了多少”，但 AMCL 不认为这个结果绝对准确，而是在每个粒子上施加不同程度的随机扰动。里程计越不可靠，运动噪声就应该越大；但具体数值需要依据机器人和仿真数据调参，不能仅凭默认值推断。

### 3.3 校正：用激光给粒子打分

当前项目使用：

```yaml
laser_model_type: likelihood_field
```

Nav2 Jazzy 支持 `beam`、`likelihood_field` 和 `likelihood_field_prob` 三种激光模型名称；当前配置选择的是 `likelihood_field`。[官方 AMCL 参数说明](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/)

#### Likelihood-field 模型

它的核心不是对每一束激光都完整地做一次“从机器人沿射线走到障碍物”的精确射线仿真，而是：

1. 在地图上为占用空间生成到最近障碍物的距离场；
2. 假设某束激光击中障碍物后，击中点离最近地图障碍物越近，概率越高；
3. 用 `sigma_hit` 控制高斯距离模型的宽度；
4. 用 `z_hit` 和 `z_rand` 混合“命中地图障碍”和“随机测量”两类概率。

Nav2 Jazzy 的构造函数会根据 `laser_likelihood_max_dist` 对地图建立距离场；传感器更新时对每个粒子的每束观测计算障碍距离，并使用高斯项与随机项累积粒子权重。[Likelihood-field 构造与距离场](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp#L27-L36)；[Likelihood-field 权重计算](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp#L38-L122)

因此，某个粒子如果放在正确位置，它投影出的激光击中点通常会靠近地图障碍物，权重变大；放在错误位置时，击中点离地图障碍较远，权重变小。

#### Beam 模型

`beam` 模型显式比较每束激光的观测距离和地图射线预测距离，并由四类混合项组成：

- `z_hit`：测量接近预期障碍物距离；
- `z_short`：测量比预期更短，可能是意外物体；
- `z_max`：测量达到传感器最大量程；
- `z_rand`：随机测量。

这些项和 `sigma_hit`、`lambda_short` 等参数在 Jazzy `BeamModel` 中保存并参与传感器函数计算。[Jazzy BeamModel](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/sensors/laser/beam_model.cpp#L27-L39)

当前项目先使用 likelihood-field 是合理的学习起点：它结构较清晰、对少量未知/异常激光更宽容。这里的“更适合作为起点”是工程判断，不是 Nav2 文档对所有机器人都作出的保证。

### 3.4 权重归一化和重采样

Jazzy 的粒子滤波器先调用传感器模型计算总权重，再将每个粒子的权重除以总权重，得到概率意义上的归一化权重，同时更新 `w_fast` 和 `w_slow` 两个运行平均值。[Jazzy 传感器更新与归一化](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/pf/pf.c#L236-L273)

接着按 `resample_interval` 决定是否重采样。重采样会偏向复制高权重粒子，淘汰低权重粒子；Jazzy 实现还使用 KD-tree 统计粒子分布，以支持 KLD 自适应采样，并在权重恶化时按 `w_fast/w_slow` 的差异加入随机位姿恢复粒子。[Jazzy 重采样实现](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/pf/pf.c#L276-L345)

相关参数的含义：

| 参数 | 当前值 | 学习含义 |
|---|---:|---|
| `min_particles` | `500` | 自适应采样允许的最少粒子数 |
| `max_particles` | `2000` | 自适应采样允许的最多粒子数 |
| `pf_err` | `0.05` | 允许的粒子群误差 |
| `pf_z` | `0.99` | 粒子群覆盖概率参数；官方说明指出 `2.33` 对应 99% 分位数的相关实现约定 |
| `resample_interval` | `1` | 每次滤波更新都允许重采样 |
| `recovery_alpha_fast` | `0.0` | 快速平均权重衰减率；为 `0` 时关闭这类随机恢复调节 |
| `recovery_alpha_slow` | `0.0` | 慢速平均权重衰减率；为 `0` 时关闭这类随机恢复调节 |

`min_particles`、`max_particles`、`pf_err`、`pf_z` 和 `resample_interval` 的默认含义由 Nav2 参数页定义；`recovery_alpha_fast/slow` 用于判断是否加入随机恢复位姿。[Nav2 AMCL 参数定义](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/)

### 3.5 从粒子群得到位姿

重采样后，AMCL 会按粒子聚类统计各个假设的权重、均值和协方差，选择最大权重假设作为当前定位结果，然后发布 `amcl_pose` 和粒子云。[Jazzy 最大权重假设计算](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L831-L864)；[Jazzy 粒子云发布](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L812-L829)

所以 `amcl_pose` 不是“某一个粒子直接当答案”，而是从粒子群的聚类和统计结果中得到的估计。若环境有多个相似走廊或房间，粒子群可能长期存在多个聚类；这也是 AMCL 需要粒子云可视化的重要原因。

## 4. `map -> odom` 到底是什么意思

### 4.1 三个核心坐标系

REP-105 对移动平台规定了以下语义：

- `base_link`：固定在机器人本体上的坐标系。
- `odom`：连续、短期平滑，但会随里程计累计漂移的世界坐标系。
- `map`：长期全局参考坐标系，定位结果应尽量不漂移，但允许因为全局校正发生跳变。

REP-105 要求最小 TF 拓扑为：

```text
map -> odom -> base_link -> sensor frames
```

并明确指出：里程计系统负责 `odom -> base_link`；定位系统先估计 `map -> base_link`，但应利用已存在的 `odom -> base_link` 计算并发布 `map -> odom`，而不是让两个系统同时发布 `map -> base_link`。[REP-105 坐标语义与 TF 拓扑](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst#relationship-between-frames)；[REP-105 Frame Authorities](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst#frame-authorities)

Nav2 的状态估计说明也采用同一分工：定位系统提供 `map -> odom`，里程计系统提供 `odom -> base_link`。[Nav2 State Estimation](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/)

### 4.2 为什么 AMCL 不直接发布 `map -> base_*`

设：

```text
T_map_base = AMCL 根据地图和激光估计的全局位姿
T_odom_base = 里程计系统提供的局部位姿
```

则应发布：

```text
T_map_odom = T_map_base × inverse(T_odom_base)
```

这样，其他模块通过 TF 查询 `map -> base_*` 时，会得到：

```text
map -> odom -> base_*
```

其中 `odom` 的短期连续性保留给控制和局部运动使用，`map` 的全局一致性由 AMCL 不断校正。AMCL Jazzy 源码在得到最大权重位姿后计算 `map -> odom`，并根据 `tf_broadcast` 决定是否广播。[Jazzy AMCL TF 更新顺序](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L646-L688)；[Jazzy `calculateMaptoOdomTransform()`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L925-L960)

因此，本项目中绝不能再用静态 TF 发布器伪造 `map -> odom`。那样会把“定位结果”变成一个永远不变的常量，无法纠正 odom 漂移，也会与 AMCL 争夺同一条 TF 边。

## 5. 常见失败原因和检查方法

下面的故障分类是根据 Jazzy 源码中的输入检查、TF 查询、生命周期逻辑和 REP-105 推导出的工程排查表；“推导”表示它是对源码行为的诊断归纳，不是 Nav2 对每个场景的逐项保证。

| 现象 | 常见原因 | 先检查什么 |
|---|---|---|
| `/map` 没有发布者 | `map_server` 没启动、没有 active，或地图 YAML/PGM 路径错误 | `ros2 topic info /map`；`ros2 lifecycle get /map_server`；检查地图文件 |
| AMCL 一直提示等待地图 | AMCL 没收到地图，或者 map QoS/话题名不匹配 | `ros2 topic info /map -v`；确认 `map_topic` 为 `map` |
| 没有 `/amcl_pose` 或粒子云 | AMCL 未 active、没有初始位姿，或还没有成功处理激光 | `ros2 lifecycle get /amcl`；检查 `/initialpose`；检查 AMCL 日志 |
| 设置初始位姿没有效果 | 消息 frame 不是 `map`，初始点在地图范围外，或 AMCL 尚未 active | `ros2 topic echo --once /initialpose`；确认 `header.frame_id: map` |
| 有 `/scan`，但 AMCL 不更新 | `scan` frame 到 `base_frame_id` 的 TF 缺失；扫描时间处没有 `odom -> base_*` | `ros2 run tf2_ros tf2_echo base_footprint <laser_frame>`；检查仿真时间和时间戳 |
| RViz 报 Fixed Frame/No transform | 固定坐标系设置错误，或 `map -> odom -> base_*` 链路不完整 | `ros2 run tf2_tools view_frames`；检查 RViz Fixed Frame 为 `map` |
| 机器人在地图上跳动/有多个位置 | 地图与 Gazebo 世界原点、分辨率或 origin 不一致；初始位姿不合理；环境对称 | 先确认地图 YAML 的 `resolution/origin`，再看粒子云是否多峰 |
| TF 树出现同一条边的多个发布者 | AMCL、静态 TF 或另一个定位节点同时发布 `map -> odom` | `ros2 topic info /tf -v`；停止其他定位/静态 TF 发布者 |
| 定位逐渐漂移 | 没有 AMCL 发布的 `map -> odom`，或 AMCL 已失去有效激光匹配 | 检查 `map -> odom` 是否动态更新、粒子权重和激光质量 |
| 定位偶发停止 | `update_min_d/update_min_a` 阈值未达到，或者消息/TF 时间同步失败 | 检查机器人是否移动、扫描时间戳和 AMCL 日志 |

几个容易忽略的实现细节：

1. AMCL 的地图订阅是 reliable + transient local；地图发布者也应保留地图，启动顺序变化时才不会因为错过唯一一条地图消息而空等。[Jazzy 地图订阅 QoS](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1297-L1306)
2. AMCL 的激光输入使用 sensor-data QoS，并用 TF MessageFilter 等待目标时间的 TF；所以“`/scan` 有数据”不等于“AMCL 已经能够使用这些数据”。[Jazzy 激光 MessageFilter](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1424-L1442)
3. `map` 的 `header.frame_id` 应与 AMCL 的 `global_frame_id` 一致。Jazzy 源码会对此发出警告；当前项目两者都应为 `map`。[Jazzy 地图处理](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp#L1311-L1346)
4. `base_frame_id` 必须与实际 TF 树一致。项目当前使用 `base_footprint`，而不是默认值也常见的 `base_link`；这个选择来自 Waffle 的实际 TF 配置。[项目 AMCL 配置](../mini_nav_bringup/config/amcl_waffle.yaml)

## 6. 和当前 `mini_nav` 项目的对应关系

### 6.1 当前启动链路

当前完整入口是：[official_localization_astar.launch.py](../mini_nav_bringup/launch/official_localization_astar.launch.py)

```text
TurtleBot3 Waffle / Gazebo
   ├── /scan
   ├── odom -> base_footprint
   └── base_footprint -> laser frame
             │
             ▼
Nav2 map_server ── /map ──┐
                          ▼
                    Nav2 AMCL
                  ▲       │
       /initialpose      ├── /amcl_pose
       from RViz         ├── /particle_cloud
                          └── map -> odom

/map ──► CostmapPublisherNode ──► mini_nav_core::AStarPlanner
                                   │
                       /goal_pose ─┘
                                   ▼
                          /mini_nav/global_path
```

启动文件通过 Nav2 官方 `localization_launch.py` 启动 `map_server`、`amcl` 和 lifecycle manager，然后启动本项目的 `CostmapPublisherNode` 与 RViz。[项目完整启动文件](../mini_nav_bringup/launch/official_localization_astar.launch.py)；[Nav2 Jazzy localization launch](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/localization_launch.py)

### 6.2 当前参数对应关系

项目配置文件是：[amcl_waffle.yaml](../mini_nav_bringup/config/amcl_waffle.yaml)

| 项目参数 | 当前值 | 含义 |
|---|---|---|
| `global_frame_id` | `map` | AMCL 的全局地图坐标系 |
| `odom_frame_id` | `odom` | 仿真里程计坐标系 |
| `base_frame_id` | `base_footprint` | Waffle 的基座坐标系 |
| `scan_topic` | `scan` | 激光输入 `/scan` |
| `map_topic` | `map` | 地图输入 `/map` |
| `laser_model_type` | `likelihood_field` | 使用距离场激光模型 |
| `robot_model_type` | `nav2_amcl::DifferentialMotionModel` | 差速运动模型 |
| `set_initial_pose` | `false` | 等待 RViz `/initialpose` |
| `tf_broadcast` | `true` | 允许 AMCL 发布 `map -> odom` |

启动文件默认使用项目安装的地图资源，并将 `use_sim_time` 传给仿真、AMCL、A* 节点和 RViz；当前地图是 `107 × 107`、分辨率 `0.05 m/格`，origin 为 `(0.724, -3.997, 0)`。[项目地图 YAML](../mini_nav_bringup/maps/turtlebot3_map.yaml)；[项目启动参数](../mini_nav_bringup/launch/official_localization_astar.launch.py)

### 6.3 当前项目中 AMCL 和自研 A* 的边界

当前 A* 使用的是你写的 `mini_nav_core::AStarPlanner`，AMCL 使用的是官方 Nav2 `nav2_amcl`。两者通过地图和 TF 配合，但职责不同：

- AMCL 负责把机器人定位到 `map`，并提供 `map -> odom`。
- `CostmapPublisherNode` 接收 `/map`，转换为项目自己的 `Costmap2D`。
- A* 接收用户给出的 `/initialpose` 和 `/goal_pose`，发布 `/mini_nav/global_path`。
- 当前 A* 还没有自动从 TF 或 `/amcl_pose` 读取机器人真实起点，也没有路径跟踪和 `/cmd_vel` 输出；因此 `/mini_nav/global_path` 仍是规划结果，不是可执行轨迹。[项目架构说明](architecture.md)

这里有一个值得尽早修正的架构点：后续路径跟踪器应该通过 TF 查询 `map -> base_footprint`，或者订阅 `/amcl_pose` 获取机器人当前位姿，而不是把 RViz 最近一次点击的初始位姿继续当作实时起点。

## 7. AMCL 和自研定位的学习路线

不建议一开始就复制整个 `nav2_amcl`。更适合本项目的拆分是：

### 阶段 A：只做 ROS 无关的运动预测

在 `mini_nav_core` 中实现一个差速运动模型：输入上一帧位姿和 odom 增量，输出带噪声的粒子位姿。先测试：

- 纯前进；
- 原地旋转；
- 前进后旋转；
- 角度归一化；
- 零运动；
- 固定随机种子下的可重复结果。

这一步对应 Nav2 Jazzy 的 `DifferentialMotionModel`，应先把坐标和噪声搞懂，再接 ROS TF。[Nav2 Jazzy 差速模型源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/motion_model/differential_motion_model.cpp)

### 阶段 B：实现地图坐标和最小激光模型

先实现一个非常简单的 likelihood-field 评分：对每个粒子取少量激光束，把击中点转换到地图格，查询该点到最近障碍物的距离，使用高斯函数得到权重。不要先追求全部 AMCL 参数；先验证“正确位姿的权重大于明显错误位姿”。Nav2 Jazzy 的 likelihood-field 源码可以作为对照，但它包含地图距离场、异常测量和工程化性能处理，不宜直接视为最小教学实现。[Nav2 Jazzy likelihood-field 源码](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/sensors/laser/likelihood_field_model.cpp)

### 阶段 C：实现粒子滤波器

完成：初始化、运动预测、权重计算、归一化、系统重采样或低方差重采样，并发布粒子云。先固定粒子数量，确认结果正确后，再加入 `min/max_particles`、KLD 自适应和随机恢复。Nav2 的 `pf.c` 展示了从传感器权重归一化到重采样的工程实现。[Nav2 Jazzy 粒子滤波源码](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_amcl/src/pf)

### 阶段 D：接入 ROS 2 话题和 TF，但先不抢 TF 所有权

自研节点先订阅 `/map`、`/scan`、`/initialpose`，发布：

```text
/mini_nav/localization_pose
/mini_nav/particle_cloud
```

同时保留官方 AMCL 发布 `map -> odom`。把自研位姿与 `/amcl_pose` 在 RViz 和日志中对比，确认收敛、协方差和丢失恢复行为。

### 阶段 E：最后才切换 `map -> odom`

只有当自研定位已经能稳定处理：初始位姿、里程计噪声、激光 TF、地图边界、无效激光、重采样退化和丢失恢复，才关闭官方 AMCL 的 `tf_broadcast`，让自研节点成为唯一的 `map -> odom` 发布者。任何时刻都不能让两个定位器同时发布同一条 TF 边。

## 8. Jazzy 与其他 Nav2 版本的注意事项

1. 本资料优先引用 `navigation2` 的 `jazzy` 分支源码；这比直接复制当前 Rolling 参数页面更适合本机 ROS 2 Jazzy。[Nav2 `jazzy` 分支](https://github.com/ros-navigation/navigation2/tree/jazzy)
2. Nav2 网站当前可访问的 AMCL 参数页位于 Rolling 文档路径，参数页会随后续发行版继续增加或调整内容；因此新参数不能只凭 Rolling 页面就假设本机 Jazzy 二进制一定支持。需要以本机 `ros2 param describe /amcl <name>` 或 Jazzy 源码的 `declare`/读取代码为准。[Rolling AMCL 参数页](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/)
3. Jazzy 分支的 `nav2_amcl` README 说明它主要是 ROS 1 AMCL 的重构移植，算法没有改变；但 ROS 2 lifecycle、QoS、参数声明、launch 和插件接口仍然是 ROS 2/Nav2 的运行时约束。[Nav2 Jazzy AMCL README](https://api.nav2.org/nav2-jazzy/html/md_nav2_amcl_README.html)
4. `localization_launch.py` 在 Jazzy 中会把 launch 参数 `map` 作为 `yaml_filename` 传给 `map_server`；项目参数文件因此只放 AMCL 参数，不应再放一个会覆盖地图路径的 `map_server.yaml_filename`，除非明确理解参数合并顺序。[Jazzy localization launch 的 map_server 参数](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/localization_launch.py#L124-L148)
5. `base_link` 是 REP-105 的标准机器人基座语义，但实际机器人可以有 `base_footprint` 等中间坐标系；关键是 TF 拓扑唯一、参数与实际 frame 一致，而不是机械地把所有机器人都改成同一个名字。[REP-105 坐标关系](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst#extra-intermediate-frames)

## 9. 建议的动手顺序

阅读本文后，建议按这个顺序在当前项目中验证，而不是直接开始大规模改代码：

1. 启动 `official_localization_astar.launch.py`，确认 `/map`、`/scan`、`/odom` 和 `map -> odom -> base_footprint` 都存在。
2. 在 RViz 设置一次 `/initialpose`，观察 `/amcl_pose` 和 `/particle_cloud` 是否出现并收敛。
3. 只移动机器人，不改变目标，观察 `odom -> base_footprint` 是否连续、`map -> odom` 是否吸收漂移。
4. 调用全局重定位服务，观察粒子是否扩散到多个自由空间区域，再用激光逐步收敛。
5. 记录 `/amcl_pose`、粒子云和 TF，建立自研定位的对照数据。
6. 先实现运动模型单元测试，再实现传感器评分，最后再写 ROS 2 定位节点。

### 核心记忆句

> 里程计负责“短期连续地猜我走了多远”，激光和地图负责“长期纠正我在全局哪里”；AMCL 用带权粒子把这两种信息融合起来，并通过 `map -> odom` 把全局定位接到连续里程计上。

## 参考来源

- [Nav2 Jazzy `nav2_amcl` API 文档](https://docs.ros.org/en/jazzy/p/nav2_amcl/)
- [Nav2 Jazzy AMCL README](https://api.nav2.org/nav2-jazzy/html/md_nav2_amcl_README.html)
- [Nav2 `jazzy` 分支 `amcl_node.cpp`](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/amcl_node.cpp)
- [Nav2 `jazzy` 分支粒子滤波器](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_amcl/src/pf)
- [Nav2 `jazzy` 分支差速运动模型](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_amcl/src/motion_model/differential_motion_model.cpp)
- [Nav2 `jazzy` 分支激光模型](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_amcl/src/sensors/laser)
- [Nav2 `jazzy` 分支 localization launch](https://github.com/ros-navigation/navigation2/blob/jazzy/nav2_bringup/launch/localization_launch.py)
- [Nav2 AMCL 参数说明](https://docs.nav2.org/rolling/configuration_and_development/configuration_guide/others/configuring_amcl/)
- [Nav2 State Estimation](https://docs.nav2.org/rolling/getting_started/navigation_concepts/state_estimation/)
- [ROS REP-105：Coordinate Frames for Mobile Platforms](https://github.com/ros-infrastructure/rep/blob/master/rep-0105.rst)
- [Fox, Burgard, Dellaert, Thrun, “Monte Carlo Localization: Efficient Position Estimation for Mobile Robots”, AAAI-99](https://aaai.org/papers/050-aaai99-050-monte-carlo-localization-efficient-position-estimation-for-mobile-robots/)
