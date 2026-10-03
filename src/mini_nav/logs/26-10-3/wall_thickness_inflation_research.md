# FAST-LIVO2 墙体过厚与导航膨胀：源码、开源项目和论文研究

调研日期：2026-10-03–04；日志目录沿用本任务的 `26-10-3`。范围：当前 `/home/a/ros2_ws/src/mini_nav` 已回退的 FAST-LIVO 建图实现。此轮仅调查、提出方案，未修改生产代码、配置、地图或启动运行中的节点。之前 GICP、证据栅格、有限首帧平面融合的修复已撤回，下面不把它当成已采用的实现。

## 建议先做什么

**推荐保留 FAST-LIVO2 前端、现有两个 launch 和自研导航核心，增加“逐帧数据记录 → 离线联合位姿优化 → 从空图重放三维占用证据 → 车高投影 → 成对保存定位图/导航图”的地图制作流程。几何优化优先验证 HKU 官方 HBA；占用更新优先以 OctoMap 官方算法作为基准。**

这是一项待验证的工程选择，依据是：本地实验已显示单帧墙很薄，但转向误差经过永久占用累积后变成厚墙；需要同时纠正跨帧几何错位和占用更新规则。只换点云下采样、只提高占用阈值或只缩小膨胀半径，都没有处理完整原因。HBA 原论文直接研究点云地图分层/一致性，输入独立扫描与初始位姿，适合先建图再导航；它是 ROS1 工程，尚不能声称可直接运行 Jazzy。[HBA 论文，§I–III](https://hub.hku.hk/bitstream/10722/331150/1/content.pdf)、[官方编译文件](https://github.com/hku-mars/HBA/blob/a0cdd474996fd9bb76888c8d1839b49b70aa0818/CMakeLists.txt)。

当前最小实验应先跑通离线 A/B，不立即替换在线建图节点。若只需导航几何，用 HBA；若同时要优化彩色地图和相机位姿，再评估 FAST-LIVO2 作者推荐的 Global-LVBA。若后续要完整实时子图/闭环，GLIM 是可研究的 ROS2 方案，但属于更大范围替换，不能当成现有 FAST-LIVO2 的即插即用后端。[Global-LVBA 官方说明](https://github.com/xuankuzcr/Global-LVBA/blob/10493a2b0f695e32db83dde49dad843a4826e090/README.md)、[GLIM 论文，§V](https://arxiv.org/html/2407.10344v1#S5)。

## 1. 本项目已经知道什么

### 1.1 原始地图增厚已经复现

此前两轮独立 Gazebo 运行中，同一段墙的原始黑色占用带，从静止参考约 11.83/11.84 cm 变为正反转向后的 16.16 cm，增厚约 4.32 cm。它是 `/fastlivo/map_2d_live`，不是 costmap 膨胀图。这里的宽度按斜墙栅格中心沿法线跨度并计入格宽计算，包含 5 cm 栅格离散化，不能当作墙的真实结构厚度。[原复现报告](wall_thickness_reproduction/report.md)、[第二轮测量](wall_thickness_reproduction/run_20261003_191428/measurement.json)。

第二轮单帧三维墙点法线散布约 4 mm，三维高度点单独累计仍会形成约 13.65 cm 的黑带。二维扫描用 Gazebo 真值位姿投影时约 11.83 cm 保持不变，用 FAST-LIVO 位姿时增长到约 16.16 cm；相对 XY 偏差峰值约 7.25 cm。这支持“跨帧投影偏差 + 累积机制”，但尚未锁定前端瞬态偏差究竟由哪个内部模块造成。[输入分离实验](wall_thickness_reproduction/run_20261003_191428/attribution.json)。

### 1.2 回退后的代码仍会永久保留错位命中

| 当前代码事实 | 直接影响 |
|---|---|
| `mapping_node.py` 把三维 `collisions` 调用 `grid.mark()` | 任何落入新格的错位点都会扩张静态障碍 |
| `/scan` 做自由射线之后，又把带噪 hit 端点 `mark()` | 二维噪声和三维高度占用混为同一静态黑图 |
| `HeightGrid.mark()` 把 `occupied=True`；`rays()` 只设置另一个 `free` 标志；`data()` 占用覆盖自由 | 后来正确的观测不会撤销先前错误黑格 |
| `GeometryStore` 每 0.1 m 体素只保留首个点 | 降采样控制密度，但缺少每帧来源与历史重建能力 |
| 当前地图分辨率 0.05 m，高度区间 0.02–0.40 m | 亚栅格波动可能跨过格边形成额外黑格 |

依据：[当前建图节点](../../mini_nav_fastlivo/mini_nav_fastlivo/mapping_node.py)、[当前栅格](../../mini_nav_fastlivo/mini_nav_fastlivo/grid.py)、[建图参数](../../mini_nav_fastlivo/config/mapping.yaml)。重复完全相同一帧 100 次不改变这张布尔图；必须是坐标变动或新格命中才会长厚，因此“点数量太多”不是充分根因。

### 1.3 墙厚与膨胀要分开测量

当前自研规划和局部 costmap 配置为 `robot_radius=0.24 m`、`safety_margin=0.02 m`，硬安全距离约 0.26 m；`inflation_radius=0.45 m` 是代价影响的外圈，不能把整圈都解释为绝对禁行。当前 `inflate()` 从固定源图的障碍格计算，不把刚产生的膨胀格再次作为源；`rebuildPlanningMap()` 从原图拷贝、融合当前原始观测后再膨胀，源码检查不支持无限链式重复膨胀。[规划配置](../../mini_nav_bringup/config/planning_costmap.yaml)、[局部配置](../../mini_nav_bringup/config/local_costmap.yaml)、[自研 inflation](../../mini_nav_core/src/map/inflation_layer.cpp)、[规划图重建](../../mini_nav_nodes/src/costmap_publisher.cpp)。

官方 Nav2 1.3.12 同样以障碍源格中心的欧氏距离生成代价，内切半径内是碰撞相关高代价，外侧为指数衰减。这里用于核对距离概念，不代表本项目已改用 Nav2 插件。本项目自研 inflation 计算中心到占据方格面积的距离，并使用外接圆加 margin；Nav2 使用格中心距离和 footprint 内切半径，因此相同数值参数不保证相同边界。[Nav2 `computeCost()`](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/include/nav2_costmap_2d/inflation_layer.hpp#L139-L156)、[官方源障碍队列](https://github.com/ros-navigation/navigation2/blob/1.3.12/nav2_costmap_2d/plugins/inflation_layer.cpp)。

查看效果时，先在导航 RViz 仅显示 `/map`，随后分别开启 `/mini_nav/planning_costmap` 和 `/mini_nav/local_costmap`；后两张采用 costmap 配色且默认叠加显示。内部 254 显示为 100（障碍），253 显示为 99（硬安全），软代价显示为 1–98；这些色带不是静态墙的实体厚度。建图预设的 `/fastlivo/map_2d_live` 才是当前原始高度图。[显示换算](../../mini_nav_nodes/include/mini_nav_nodes/costmap_display.hpp)、[规划图说明](../../docs/planning_costmap.md)。

**工程推断：**错误黑墙向通道内部多占多少，后续安全区边界就会相应向内推进多少；两边墙一起侵入时，可行走净宽进一步缩小。4.32 cm 的黑带增长不等于单边通道边界一定侵入 4.32 cm，要测量增长朝向。不能用缩小车体硬半径掩盖地图错位；地图正确后才评估软代价范围和衰减参数。

## 2. 一手资料与源码核查

### 2.1 FAST-LIVO2/VoxelMap：局部平面估计不等于全局导航地图

FAST-LIVO2 论文 §V 介绍统一体素地图中的几何平面和视觉 patch，§IX-B 明确比较的是无闭环里程计，§XI 将闭环和滑窗优化列为后续工作。因此不能把“已有 scan-to-map”和“有全局历史位姿修正”混为一谈；论文中的视觉 raycast 是观测可见性机制，也不是占用栅格的 miss 清除。[FAST-LIVO2 原论文](https://arxiv.org/html/2408.14035v2)。

实际源码中 `StateEstimation()` 后对新世界点调用 `UpdateVoxelMap()`；`mapSliding()` 删除窗口外体素，解决局部图管理，而不是把历史全局 PCD 按修正轨迹重新拼接。导出的世界扫描与其内部平面结构也是不同数据产品。[`LIVMapper.cpp`](https://github.com/hku-mars/FAST-LIVO2/blob/0d2c0346107b75b59934975adec9a6eeeb913c64/src/LIVMapper.cpp#L345-L435)、[`voxel_map.cpp`](https://github.com/hku-mars/FAST-LIVO2/blob/0d2c0346107b75b59934975adec9a6eeeb913c64/src/voxel_map.cpp#L924-L970)。

VoxelMap 论文的概率模型用于平面参数不确定度和位姿匹配，并非每个空间体素的 occupied/free 后验。借鉴平面匹配需要正确关联与退化门控，不能把所有墙点强行投到第一帧平面。[VoxelMap 原论文](https://arxiv.org/html/2109.07082v3)。

FAST-LIO `map_incremental()`/ikd-Tree 的中心近点保留、增量插入与局部删除可以控制点密度；它不是闭环，不能保证漂移后的点仍落入同一体素。[FAST-LIO 官方源码](https://github.com/hku-mars/FAST_LIO/blob/7cc4175de6f8ba2edf34bab02a42195b141027e9/src/laserMapping.cpp#L427-L473)。

### 2.2 OctoMap：可更新证据是基础，但默认部署不能自动修好

OctoMap 论文 §3.2/§4 用 log-odds 融合命中和沿真实射线穿过的自由体素，并限制置信度上下界；它明确区分占用、自由、未知。概率更新能让有后续观测的噪声占用反转，而不是永久布尔 OR。[OctoMap 论文原文](https://www.arminhornung.de/Research/pub/hornung13auro.pdf)。

`insertPointCloud()` 先得到整帧 `free_cells` 和 `occupied_cells` 集合；`computeUpdate()` 删除两者交集，命中优先，然后每个体素每帧更新一次。因此应使用批量点云插入思路，不能每个重复点或每条 ray 反复增票。[官方 `OccupancyOcTreeBase.hxx`](https://github.com/OctoMap/octomap/blob/b5f9a9598f734609974d1086a785c54d25ec8763/octomap/include/octomap/OccupancyOcTreeBase.hxx#L86-L264)。

官方默认 `occupancy=0.5`、`hit=0.7`、`miss=0.4`，clamp 约 `[0.1192,0.971]`。未知初值 0.5 遇到一次 hit 即可超过默认占用阈值；错误位姿反复命中还会形成高置信度鬼墙。换成默认 OctoMap 并不能代替位姿优化。[默认参数源码](https://github.com/OctoMap/octomap/blob/b5f9a9598f734609974d1086a785c54d25ec8763/octomap/src/AbstractOccupancyOcTree.cpp#L38-L48)。

官方 ROS2 `octomap_server` 有 `occupancy_min_z`/`occupancy_max_z`，按体素实际空间范围判断高度区间相交，再做二维投影；`update2DMap()` 在当前投影中 occupied 覆盖 free。这是三维柱的保守 OR，不是对整根柱做平均。依旧要防止不同高度的残留 occupied 累加横向黑带。该服务器默认完整重投影；增量模式也先清空更新 BBX 后重投影，因此它的二维投影不是本项目永久布尔 OR。其 free 只在当前柱没有 occupied 时填值，并不证明整车高柱全部被观测为自由。[ROS2 服务器源码](https://github.com/OctoMap/octomap_mapping/blob/f79da9a9a1fcdf82e72dab4df288d6cc27c6e163/octomap_server/src/octomap_server.cpp#L679-L709)、[`update2DMap()`](https://github.com/OctoMap/octomap_mapping/blob/f79da9a9a1fcdf82e72dab4df288d6cc27c6e163/octomap_server/src/octomap_server.cpp#L1277-L1330)。

直接启动默认服务器还有两项具体接入风险：

- `/fastlivo/cloud_world` 的 frame 是 `camera_init` 世界系，而 server 根据 cloud frame 的 TF 位置取传感器原点；直接 remap 到 `cloud_in` 会从世界原点射线。应发 LiDAR 本体系 cloud 配同期 `T_map_lidar`，或直接调用 OctoMap core 并显式传真实 origin。[服务器插入源码](https://github.com/OctoMap/octomap_mapping/blob/f79da9a9a1fcdf82e72dab4df288d6cc27c6e163/octomap_server/src/octomap_server.cpp)。
- server 的 `filter_ground_plane` 默认 false，`ground_filter.distance` 默认 0.04 m；直接启用默认4cm地面剔除可能吞掉3cm箱体/门槛。当前碰撞起始高度2cm，必须测试更精确地面分类、地面端点的体素高度跨界；可靠地面返回仍贡献真实 ray free，却不能误作 collision endpoint。不能仅把 `projected_map` 默认参数换进来就宣称高度碰撞语义正确。[同一官方参数与地面处理源码](https://github.com/OctoMap/octomap_mapping/blob/f79da9a9a1fcdf82e72dab4df288d6cc27c6e163/octomap_server/src/octomap_server.cpp)。

对本项目的要求是：先用完整有效三维返回和真实 LiDAR 原点建立相应空间的 hit/miss，再裁取车高占用层。不能先丢掉地面/高处返回导致射线自由证据缺失；更不能用 0.132 m 的二维 scan 穿过一点，就把 0.02–0.40 m 整柱擦成自由。没有看到的低矮障碍高度仍是未知。候选/确认门限需单独验证薄障碍，未确认点不能随意当白色自由。

### 2.3 Cartographer：局部子图、概率插入与全局修正的职责分开

Cartographer 原论文 §III–V 使用有限局部子图进行扫描匹配，已结束子图参与闭环和全局位姿优化；概率格插入使用互斥 hit/miss 集合。其思想是保持局部几何一致并另行处理全局累积误差。[论文作者官方页面及原文](https://research.google/pubs/real-time-loop-closure-in-2d-lidar-slam/)。

源码中 `ProbabilityGrid::ApplyLookupTable()` 的 update marker 阻止同一轮重复更新，插入器在 hit 后尚不结束本轮更新，使同帧 hit 优先于 miss；`FinishUpdate()` 再清除标记。[`probability_grid.cc`](https://github.com/cartographer-project/cartographer/blob/8d7d94daaea1ef6fe3db908ac95b8fe2714e9ff9/cartographer/mapping/2d/probability_grid.cc#L54-L65)、[range inserter](https://github.com/cartographer-project/cartographer/blob/8d7d94daaea1ef6fe3db908ac95b8fe2714e9ff9/cartographer/mapping/2d/probability_grid_range_data_inserter_2d.cc#L56-L64)。

这适合作为“插入协议”和“地图可重建”的参考。本次不建议为了修墙厚直接接入整套 Cartographer；其二维 scan 模型也不能替代三维低障碍保护。

### 2.4 HBA/BALM：优先联合优化帧位姿，再重建几何

HBA 论文指出，PGO 优化相对位姿约束不必然让所有表面一致；LiDAR BA 直接优化多帧共同平面的点到面一致性，HBA 用局部层级 BA 加上顶层/跨层 PGO降低计算规模。它修改的是各帧刚体位姿，不是把一张黑白图片统一腐蚀。HBA 并不提供自动全局闭环候选检测；收益仍依赖帧间重叠、初始估计和退化检查，不能保证仅输入任意漂移轨迹就正确闭环。[HBA 论文，§I–III](https://hub.hku.hk/bitstream/10722/331150/1/content.pdf)、[BALM 论文原文](https://hub.hku.hk/bitstream/10722/301528/1/content.pdf?accept=1)。

官方 HBA 代码输入 `pcd/0.pcd...` 和初始扫描位姿文件，`hba.cpp` 完成分层优化后调用 `pose_graph_optimization()`。注意 `pose.json` 名字虽叫 JSON，`mypcl.hpp` 实际按空白分隔 `tx ty tz qw qx qy qz` 读取；它无时间列。导出会覆盖该文件，并按第一帧重新设置原点。还存在 `while(!eof())` 读取及四元数分量逐项赋值的健壮性风险，适配层必须验证帧数、规范四元数和原点，不可直接裸喂生产数据。[优化入口](https://github.com/hku-mars/HBA/blob/a0cdd474996fd9bb76888c8d1839b49b70aa0818/source/hba.cpp#L495-L518)、[格式与导出](https://github.com/hku-mars/HBA/blob/a0cdd474996fd9bb76888c8d1839b49b70aa0818/include/mypcl.hpp#L51-L69)、[`write_pose()`](https://github.com/hku-mars/HBA/blob/a0cdd474996fd9bb76888c8d1839b49b70aa0818/include/mypcl.hpp#L140-L165)。

实际接入应把优化输入复制到独立目录，保存原始轨迹不可变，初始首帧正规化为单位姿态并记录坐标变换；若移植写出函数，先算完整 `q_new=q0.inverse()*q` 再一次赋值和 normalize，读取也需检查每次提取成功才增加记录；加入带非零首帧 yaw 和末尾换行的测试。核查读写后恢复 `map` 坐标和 ground 对齐。源码的 `visualize.launch` 与优化分开，不能把“优化结束”当成已有完整二维可导航图。

HBA 官方案例是 ROS1 Noetic/Melodic、PCL/Eigen/GTSAM；可先用隔离的 ROS1 环境文件式运行验证数据收益，再把数值核心做独立 CLI/ROS2 适配。是否能无 ROS1 地快速抽出核心、Jazzy 依赖兼容、内存和耗时，均未构建验证。[编译依赖](https://github.com/hku-mars/HBA/blob/a0cdd474996fd9bb76888c8d1839b49b70aa0818/CMakeLists.txt)、[上游使用约束](https://github.com/hku-mars/HBA/blob/a0cdd474996fd9bb76888c8d1839b49b70aa0818/README.md)。

### 2.5 Global-LVBA：彩色地图更匹配，但完整导航图还需自己重放

FAST-LIVO2 官方 README 直接链接 `xuankuzcr/Global-LVBA`。源码实际包含 `runLidarBA()` 的窗口/全局 BALM 优化和视觉优化流程，因此不仅是渲染教程。[FAST-LIVO2 官方链接](https://github.com/hku-mars/FAST-LIVO2/blob/0d2c0346107b75b59934975adec9a6eeeb913c64/README.md)、[Global-LVBA 数值流程](https://github.com/xuankuzcr/Global-LVBA/blob/10493a2b0f695e32db83dde49dad843a4826e090/src/lvba_system.cpp#L312-L406)。

它要求 `all_pcd_body/<timestamp>.pcd + lidar_poses.txt`、`all_image/<timestamp>.png + image_poses.txt` 和相机参数/匹配资料；默认 `loadDataset()` 即使关闭视觉优化仍先加载图片和相机位姿。源码把 PCD 时间排序后按索引关联位姿，帧数不等的检查被注释；适配层要自行校验数量和时间，不能依赖上游自动安全匹配。[数据加载](https://github.com/xuankuzcr/Global-LVBA/blob/10493a2b0f695e32db83dde49dad843a4826e090/src/dataset_io.cpp#L68-L74)、[逐帧对应](https://github.com/xuankuzcr/Global-LVBA/blob/10493a2b0f695e32db83dde49dad843a4826e090/src/dataset_io.cpp#L241-L283)。

实际有 `colored_merged_before/after.pcd` 导出，但彩色图按相机可见像素取点；只把该彩色 PCD 直接投影黑墙会漏掉相机 FoV 外可撞障碍。导航图应从优化后的完整原扫描重建，彩色图只作为独立展示资产。[彩色导出](https://github.com/xuankuzcr/Global-LVBA/blob/10493a2b0f695e32db83dde49dad843a4826e090/src/lvba_system.cpp#L2060-L2122)。

相关原论文 LVBA（Li 等，2024）描述先 LiDAR BA 再带平面先验的视觉 BA，与“先优化几何再处理彩色”的路线一致。论文作者与该 `Global-LVBA` 仓库贡献者并非完全相同，不能宣称这就是同一个逐行复现实现。[LVBA 原论文，§II](https://arxiv.org/html/2409.10868v1)。

### 2.6 闭环参考与 ROS2 仓库筛选

| 项目 | 已核查能力 | 与本项目的关系和限制 |
|---|---|---|
| [gisbi-kim/FAST_LIO_SLAM](https://github.com/gisbi-kim/FAST_LIO_SLAM/tree/10eab538db70678f41aaa24247d16831ca921374) | FAST-LIO + Scan Context + GTSAM PGO；SC-PGO 保存局部关键帧，利用优化位姿合并地图 | ROS1。适合借鉴“前端独立 + 后端独立 + 保存局部帧”；不是已经验证的 FAST-LIVO2/Jazzy适配 |
| [rohrschacht/FAST_LIO_SLAM_ros2](https://github.com/rohrschacht/FAST_LIO_SLAM_ros2/tree/705805ddcd327bb07b5634a29df2f86347f9a3c4) | 所核默认提交构建的是 FAST-LIO ROS2 `fastlio_mapping`；有 `/map_save` | 该快照没有构建 SC-PGO/GTSAM/Scan Context；仓库名字不能证明含完整闭环，不作为完整 SLAM 直接候选 |
| [koide3/glim](https://github.com/koide3/glim/tree/2262aafa2369acff6afab2268a7743aec7a7fa49) / [glim_ros2](https://github.com/koide3/glim_ros2) | 局部多帧优化、子图全局匹配误差优化；`export_points()` 按修正子图位姿重建点 | 是现成 ROS2 全局建图候选，研究价值高；输入点云/IMU契约、CPU/GPU和GTSAM依赖仍需本机验证。替换前端会扩大本次范围 |
| [OctoMap ROS2](https://github.com/OctoMap/octomap_mapping/tree/f79da9a9a1fcdf82e72dab4df288d6cc27c6e163) | 三维 hit/miss 更新、未知空间、高度区间二维投影 | 最适合作为占用重放基准，不能纠正前端漂移，默认阈值不保证抑制鬼墙 |
| [HBA](https://github.com/hku-mars/HBA/tree/a0cdd474996fd9bb76888c8d1839b49b70aa0818) | 文件式多帧几何联合优化，无图像依赖 | 首选离线几何实验；ROS1、边界健壮性与小场景参数需处理 |
| [Global-LVBA](https://github.com/xuankuzcr/Global-LVBA/tree/10493a2b0f695e32db83dde49dad843a4826e090) | FAST-LIVO2 后的几何/视觉地图一致性处理和彩色导出 | 彩色图需求的第二阶段；ROS1/Ceres/SiftGPU/图像输入，不能直接作为完整障碍图 |

上表不以 star 或仓库名字推断维护状态。固定提交是本轮实际源码核查快照，不是本机测试通过版本。FAST_LIO_SLAM 的关键帧导出/PGO地图见 [SC-PGO 源码](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/10eab538db70678f41aaa24247d16831ca921374/SC-PGO/src/laserPosegraphOptimization.cpp#L623-L776)、[重建脚本](https://github.com/gisbi-kim/FAST_LIO_SLAM/blob/10eab538db70678f41aaa24247d16831ca921374/SC-PGO/utils/python/makeMergedMap.py)；ROS2 同名分支的核查依据是 [CMake](https://github.com/rohrschacht/FAST_LIO_SLAM_ros2/blob/705805ddcd327bb07b5634a29df2f86347f9a3c4/CMakeLists.txt) 和 [map_save](https://github.com/rohrschacht/FAST_LIO_SLAM_ros2/blob/705805ddcd327bb07b5634a29df2f86347f9a3c4/src/laserMapping.cpp#L1114-L1128)。GLIM 的依据是 [论文 §V](https://arxiv.org/html/2407.10344v1#S5) 和 [真实导出函数](https://github.com/koide3/glim/blob/2262aafa2369acff6afab2268a7743aec7a7fa49/src/glim/mapping/global_mapping.cpp#L638-L687)。

### 2.7 备选：TSDF 表面融合

Voxblox/VDBFusion 的 TSDF 用有符号距离和权重融合表面，提取零交叉表面能抑制相同表面上的测量噪声。它是值得单独 A/B 的表示方法，但需要已经对齐的点云和真实射线原点，本身不是位姿纠错后端。[Voxblox 论文](https://arxiv.org/pdf/1611.03631)、[VDBFusion 作者原文，§4](https://www.ipb.uni-bonn.de/pdfs/vizzo2022sensors.pdf)、[VDBFusion 输入契约](https://github.com/PRBonn/vdbfusion/blob/1b22ae8d38db2faab00e9036bef497ddd5f3e377/src/vdbfusion/vdbfusion/VDBVolume.h)、[融合实现](https://github.com/PRBonn/vdbfusion/blob/1b22ae8d38db2faab00e9036bef497ddd5f3e377/src/vdbfusion/vdbfusion/VDBVolume.cpp)。

不能把整条 `sdf_trunc` 截断带都投成 occupied，那相当于主动制造一条厚障碍带。若用于导航，应按权重门限提取实际表面、禁止未知区域补洞、保留未知空间策略，并验证 3 cm 箱体/细柱是否会被平滑吞掉。VDBFusion `ExtractTriangleMesh` 默认补洞需要特别检查。[零交叉与提取](https://github.com/PRBonn/vdbfusion/blob/1b22ae8d38db2faab00e9036bef497ddd5f3e377/src/vdbfusion/vdbfusion/MarchingCubes.cpp)。

Voxgraph 提供 SDF 子图位姿图，但官方说明本身没有闭环检测，需要外部闭环输入，而且是 ROS1/catkin；因此不是现成完整 ROS2 闭环定位器。[Voxgraph 官方说明](https://github.com/ethz-asl/voxgraph/blob/bd802b5aee54e68e28c94d71cc0fbd7bdda98bd6/README.md)。当前主方案先验证 HBA + 三维证据；TSDF 放第二条实验支线，不与已撤回的首帧强制平面化混为同一方案。

## 3. 最小可落地方案

### 阶段 A：补齐可重放数据，不改变导航

1. 建图时记录完整去畸变局部扫描、匹配测量时刻的位姿、LiDAR-to-IMU、IMU-to-base、地面/重力参考，以及可选相机图像和标定；每次新建独立 session 目录。
2. 当前部署的 ROS2 FAST-LIVO2 已有 `pcd_save.type=1`：保存的是经外参转换的 **IMU/body** 局部点，不是 `base_footprint` 或裸 LiDAR 坐标。使用对应 `T_world_imu`；若优化库要求裸 LiDAR 坐标，再按外参成对转换点和位姿。[本地部署源码](../../../fastlivo2_deploy/FAST-LIVO2/src/LIVMapper.cpp)。
3. 当前仿真 `pcd_save_en=false,type=0`、`img_save_en=false`，旧地图包没有完整每帧扫描、来源和轨迹；需重新采集。旧 `geometry.pcd` 是每体素首点的聚合结果，不能逆推出所需历史，更不能充当逐帧原始输入。[当前仿真配置](../../../fastlivo2_deploy/fastlivo_sim/config/fastlivo.yaml)、[地图包实现](../../mini_nav_fastlivo/mini_nav_fastlivo/bundle.py)。
4. 不直接盲改开关：本地前端写 `ROOT_DIR/Log/pcd` 和 `Log/image`，需改造成独立 session、不可覆盖旧资料的导出适配。LIO 和 VIO 的时间不同，不能按消息接收顺序硬凑成一帧。录制完整 rosbag 和元数据可作为兜底；外参/时间对应必须有读回检查。

输出：可重复重放的 session；同一帧索引、时间、点数和位姿一一对应。日志不使用 Gazebo 真值修正生产数据，真值只用于独立验收。

### 阶段 B：先验证联合几何优化收益

1. 将原始 session 转为 HBA 的文件契约，保持原始数据只读；所有库输入转换写入临时工作目录。
2. 先在短程“静止 + 正反旋转 + 往返”数据上优化，首帧固定、检查刚体矩阵/四元数/帧数和原点变换。
3. 做三条同数据支路：原位姿 + 原 OR（复现基线）、原位姿 + 新占用模型（仅表示变化）、优化位姿 + 相同新模型（完整方案）。这样才能区分后端和占用规则各自收益。
4. 对优化输出检查观测点残差、墙面分层、退化方向和轨迹跳变。重复走廊/单墙约束不足，联合优化也可能错误关联，不能只看算法返回“收敛”。失败保留诊断，不导出 completed 地图。

不能照搬 HBA 默认大场景 `voxel_size=4.0 m`、0.1 m 优化采样值当导航建图分辨率。优化匹配可有独立粗采样；最终高度障碍重放仍使用完整有效扫描，确保小物体不因优化抽样从图中消失。

### 阶段 C：从空图重放三维概率证据

1. 对每帧完整返回，用优化后的姿态变换到统一 `map`；射线原点使用真实 LiDAR 原点。若局部点坐标是 IMU，射线原点仍是该帧 IMU 坐标中的 LiDAR 外参平移，不能随手用 `[0,0,0]`。本场景 `T_imu_lidar` 的平移为 `[0.032,0,0.232]`、旋转单位阵，优化后的原点为 `T_map_imu * [0.032,0,0.232,1]`；`T_imu_base_footprint=[0.032,0,-0.078]` 是另一条外参，不能混用。
2. 每帧 occupied/free 三维体素去重，hit 优先；同帧密集点不计作多份独立证据。只对真实穿过的体素加入 miss；无回波/最大量程只在传感器语义确实允许时用于自由证据。
3. 参数作为待标定值：可先对比内部三维分辨率 0.02/0.03/0.05 m，二维输出 0.05 m；选择的成本由算力和低障碍验收决定。`hit=0.7/miss=0.4` 可作上游基准，最终静态占用确认还应检查跨帧/视角支持和占用门限。没有理由把“至少两帧”当成已证明最优。
4. 高度筛选依据统一重力/地面参考；三维更新后才生成二维图。occupied 表示碰撞高度内有可靠障碍。某高度 observed-free 不能代表整柱 free；完全未知或无法证明足够覆盖的区域保持 unknown，低障碍和未确认候选需保守处理。
5. 二维 scan 命中保留运行期局部障碍，不再无条件写入静态高度图；辅助可观测区域与静态几何证据分开存，不能用其免费清整柱。高位雷达未观测地面附近时，需额外低位传感器或明确场景假设，算法不能凭空证明安全。

这阶段复用 OctoMap 核心/协议即可，不要求引入其全部 ROS 节点或让生产代码依赖 demo 包。已有聚合图缺失历史来源，不能对它“反向射线清除”重建可信自由空间。

### 阶段 D：成对导出，保持两个 launch

建图 launch 提供在线预览和 session 记录；停车结束后由独立 CLI/任务完成离线优化与重建。可以后来加 RViz 中“生成最终地图”入口，但先使文件流程可审查、可重放。导航 launch 只加载通过验收的新地图包，继续人工初值 + 既有只读定位/导航；不启动离线优化或建图节点。

`geometry.pcd` 和 `navigation.pgm/yaml` 从同一优化历史生成，保存统一坐标变换、外参、参数、版本、轨迹和内容哈希，原子生成新目录。原彩色/彩虹展示图仍可保留为独立资产，不把展示点大小或显示累计作为导航占用依据。旧地图目录保持原样。

导航仍要求 `map -> odom` 唯一动态发布者；不能给定位图保持旧坐标、只给二维图移动墙；也不能给前端临时在线姿态误差和地图坐标各加一份修正。

## 4. 文件级影响

以下是建议，不是本轮已改动：

| 文件/模块 | 建议改动或复用 |
|---|---|
| `mini_nav_fastlivo/.../mapping_node.py` | 加 session 数据记录和 source/status，预览与最终图分开；二维 noisy hit 不再直接进最终静态图；录制/导出后台任务不能阻塞 ROS 回调 |
| `mini_nav_fastlivo/.../grid.py` | 最终构建器采用可重放的三维 hit/miss 证据与高度投影；`GeometryStore` 只作最终输出降采样，不再当不可变历史源 |
| `mini_nav_fastlivo/.../geometry.py` | 复用 SE3 检查、点变换、自身过滤、高度/地面逻辑；复查统一地面坐标与实际射线 origin |
| `mini_nav_fastlivo/.../bundle.py` | 版本化增加原始session/优化轨迹元数据、双图一致性、结果状态和校验；兼容旧包，不覆写旧图 |
| 新增 `tools/export_mapping_session.py`、`tools/optimize_mapping.py`、`tools/build_navigation_bundle.py`（拟名） | 文件式离线步骤；HBA adapter 负责格式/帧数/首帧/外参转换，避免把ROS1依赖泄露到导航节点 |
| `fastlivo2_deploy/FAST-LIVO2/src/LIVMapper.cpp`、`fastlivo_sim/config/fastlivo.yaml` | 仅若本体现有导出能力不足才改路径/导出契约；保持当前前端算法和已验证时间戳同步补丁，固定版本 |
| `mini_nav_fastlivo/config/mapping.yaml`、CMake/package | 新参数显式类型/限制；独立离线依赖与构建；保留在线节点轻量 |
| `fastlivo_mapping.launch.py` / `fastlivo_navigation.launch.py` | 复用双入口，增加记录/最终包参数。离线工具可不占第三个日常运行launch |
| `registration.py` / `localization_node.py` | 复用只读 prior 定位与失效门控；本轮不再次插入已撤回的在线逐帧GICP矫正 |
| mini_nav_core A*、Action、控制、velocity_guard、PS5/RViz面板、Gazebo模型 | 复用；仅增加验收/显示原图与硬软代价区分，暂不修改车体安全距离或导航核心算法 |

## 5. 不应采用的捷径

- **整图腐蚀/开运算**：会让墙变瘦，却可能删除椅腿、细柱、小箱体和门边；没有恢复位姿和自由证据。不给这种处理当作默认安全地图。
- **把所有墙投到首帧平面**：可能把真实不同墙面、斜墙或旁边低障碍合并；之前撤回的有限融合仅刚低于原宽度阈值，不能作为成熟全局一致性方案。[撤回前证据报告](wall_thickness_fix_report.md)。
- **单纯 voxel downsample/随机删点/滤孤立体素**：减少点密度不等于纠正跨体素鬼墙；孤立细障碍可能是真实障碍。
- **默认 OctoMap + 更高阈值**：反复错位 hit 仍会被确认；抬阈值又可能漏掉短暂可见的小障碍。
- **简单时间衰减清静态图**：没有新观测就删障碍等于把未看见当自由；动态局部层衰减与静态图确认应分开。
- **只给最后整体 PCD 做一次 ICP**：已经混合的多时刻分层不能被一个全局刚体变换同时纠正；需要逐帧来源和独立位姿变量。
- **缩小硬车体半径/把 unknown 当 free**：可能恢复路径，却降低碰撞保护，不解决错误黑墙。

## 6. 验收标准：要证明通道恢复，而非刚好过线

之前撤回方案约 6.83 → 9.33 cm，增加 2.498 cm，仅略低于 2.5 cm 阈值；每轮独立墙线拟合与离散化都可能让指标恰好跨线。因此新方案不能仅用“占用带中位数增长 < 半格”作为通过依据。[之前测试报告](wall_thickness_fix_report.md)。

建议保持现有硬 0.26 m 和软 0.45 m 参数不变，在同一原始session上 A/B，并执行至少三次独立现场重复：长时间静止、多圈正反旋转、前后往返、回到同一区域、窄门通行。固定独立真值/参考墙面，不让每个阶段重新拟合自己的参考墙来掩盖偏移。

| 验收对象 | 指标/必要检查 |
|---|---|
| 三维几何 | 当前单帧与累计完整图的法向残差 P50/P95/P99、重复表面层数、优化前后轨迹与残差；检查退化方向、跳变与错误关联 |
| 原始二维静态图 | 绝对占用带、向可行走区侵入距离 P95/P99、黑格增长方向、窄门净宽；5cm离散化误差单列。目标先按真实墙/网格理论包络设定，再固定门限 |
| 膨胀与规划 | 分别测 rawmap、只静态膨胀、加入当前raw观测后膨胀；检查硬安全区边界不超出参考障碍加0.26m及既定格误差，软圈衰减符合配置。导航路径须有真实 clearance |
| 小物体和未知 | 3cm低箱、细柱、椅腿、墙旁物体、斜墙/曲面保留；高位ray不擦低障碍，单高度scan不擦整柱；未观测区域不误白 |
| 地图包一致性 | `geometry.pcd` 与二维图同一优化轨迹/坐标系，读回外参/原点一致；停机重启加载后人工初值定位和导航可完成，文件哈希保持不变 |
| 运行工程 | 录制不丢关键时刻、不阻塞回调；离线结果状态明确，失败不输出completed包；单 `map->odom`、单 `/cmd_vel`；内存/耗时随地图规模测量 |

**尚未验证：**HBA/Global-LVBA/GLIM 在本机的构建与运行、离线数据适配、最终墙厚、薄障碍保护、长距离闭环重访与实车效果。本报告研究的是可验证的路线，不把论文的准确率或官方展示结果当成 mini_nav 的实测成绩。

## 7. 版本记录与阅读顺序

本轮通过公开仓库 `git ls-remote` 固定提交并读取实际源码（GitHub API限流后使用公开源码快照），不代表它们是同一时间发布或已通过本机测试：

| 仓库 | 核查提交 |
|---|---|
| hku-mars/FAST-LIVO2 | `0d2c0346107b75b59934975adec9a6eeeb913c64` |
| hku-mars/FAST_LIO | `7cc4175de6f8ba2edf34bab02a42195b141027e9` |
| hku-mars/HBA | `a0cdd474996fd9bb76888c8d1839b49b70aa0818` |
| xuankuzcr/Global-LVBA | `10493a2b0f695e32db83dde49dad843a4826e090` |
| OctoMap/octomap | `b5f9a9598f734609974d1086a785c54d25ec8763` |
| OctoMap/octomap_mapping ros2 | `f79da9a9a1fcdf82e72dab4df288d6cc27c6e163` |
| cartographer-project/cartographer | `8d7d94daaea1ef6fe3db908ac95b8fe2714e9ff9` |
| gisbi-kim/FAST_LIO_SLAM | `10eab538db70678f41aaa24247d16831ca921374` |
| rohrschacht/FAST_LIO_SLAM_ros2 | `705805ddcd327bb07b5634a29df2f86347f9a3c4` |
| koide3/glim | `2262aafa2369acff6afab2268a7743aec7a7fa49` |
| PRBonn/vdbfusion | `1b22ae8d38db2faab00e9036bef497ddd5f3e377` |

建议先读本地复现报告和 HBA 论文 §I–III，再读 OctoMap 更新/高度投影源码；需要彩色图时读 Global-LVBA 数据加载和完整/可见点导出差异；最后再评估 GLIM 全局子图方案和 TSDF 支线。任何实现都应先交出同数据 A/B 的几何与净宽证据，再启用最终地图替换。
