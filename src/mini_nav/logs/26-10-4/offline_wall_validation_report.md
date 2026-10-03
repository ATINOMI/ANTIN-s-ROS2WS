# 墙厚问题：Git 存档与离线候选图验证

日期：2026-10-04。当前代码分支 `experiment/fastlivo_offline_wall_map`。改动前存档提交 `42551e8`，标签 `archive/mini_nav_before_offline_wall_20261004`；包含 mini_nav、FAST-LIVO 部署补丁、PS5 接入和研究报告源码，不包含地图、运行日志、压缩包或无关子仓库的改动。

## 可交给用户验收的结果

已实现并实际运行“完整去畸变扫描记录 → 三维概率射线重放 → 车高投影 → 成对地图保存”。默认采用原 FAST-LIVO2 融合位姿和 2cm OctoMap 体素，二维地图仍为5cm；硬安全距离0.26m、软代价外圈0.45m保持原样。没有恢复上次撤回的在线 GICP/首帧平面吸附。

最终候选图：`/home/a/ros2_ws/maps/fastlivo2_offline/map_fbb572df42484633`。索引保存在 [acceptance.json](../../../../maps/fastlivo2_offline/acceptance.json)。它来自749帧新数据，包括静止、正反旋转和三轮前后往返。候选自身静止到最终阶段的墙带中位宽度没有增加，但向通道内的误占仍有变化，不能声称已完全消除定位漂移或墙体误差。

[三轮中含位移一轮的地图对比图](offline_wall_validation/comparison_final.png)。新流程作用于停止建图后生成的最终图；在线预览和原 `/fastlivo/save_nav_map` 服务仍是原始布尔累积图。

## 已运行的 A/B 与选择依据

先实际编译官方 HBA 的文件式 CLI，运行109个关键帧的两层 BA/PGO，再用优化后的轨迹插值校正并重放全部540帧。最大校正约0.94cm、0.213°；第一轮3cm概率体素下，原位姿图墙带中位宽度11.83cm，HBA图14.34cm，几何参考残差P90由1.05cm变为1.12cm，未改善。因此 HBA 保留为显式 `--backend hba` 对照，默认不启用，也未把它宣称为已经成功的修复。

3cm概率体素在第二轮也未改善墙带，随后按研究计划对比2cm体素。以下三轮使用同样的2cm参数，保留每轮初始参考线，未随地图重新拟合：

| 独立采集 | 旧在线静止图 | 旧在线最终图 | 新概率最终图 | 新概率自身静止图 |
|---|---:|---:|---:|---:|
| 1：正反旋转 | 11.83cm | 14.34cm | 11.83cm | 未另行重放 |
| 2：正反旋转 | 11.83cm | 13.66cm | 11.83cm | 11.83cm |
| 3：旋转及三轮往返 | 11.83cm | 14.33cm | 11.83cm | 11.83cm |

这些数字是5cm栅格、沿墙10cm分段的占用带跨度加格宽，包含离散化与斜墙效应，**不是实体墙厚度**。原图与新图还采用不同数据表达：旧图融合下采样3D点与带噪2D命中，新图使用完整3D扫描的概率证据；实验不把全部收益归因于单一因素。

另从 Gazebo `turtlebot3_world` 碰撞网格内壁提取固定参考，只在首次位姿做一次坐标配准，测量朝通道内侧的占用格中心P95：

| 采集 | 旧在线最终图 | 新概率最终图 |
|---|---:|---:|
| 1 | 5.22cm | 3.52cm |
| 2 | 5.07cm | 5.10cm |
| 3 | 5.85cm | 5.59cm |

第二轮这一指标没有改善，第三轮新图相对自身静止阶段也有约2cm侵入增长。它说明“宽度没长”不能替代安全边界验收；参考采用网格XY投影和一次初始配准，含微小姿态/投影近似。表中是格中心距离，格面积边界还需加约3.42cm法向半格包络，之后才是0.26m车体安全圈。未自动验收窄门净宽、长时间静止或长距离闭环，交由用户继续看实际通道和导航效果。

详细数据：[第一轮](offline_wall_validation/comparison_1_resolution02.json)、[第二轮](offline_wall_validation/comparison_2_resolution02.json)、[第三轮](offline_wall_validation/comparison_3_resolution02.json)。原始扫描、逐帧位姿、点数、时间索引和SHA256保存在各 `run_*/display_maps/sessions/session_*`；优化/重放工作目录位于 `maps/fastlivo2_offline/experiment_*`，旧图不覆盖。

## 工程问题与验证结果

- 现有 `dense_map_en=false` 导致世界显示扫描已下采样。新增 `/fastlivo/cloud_body`，保存LIO阶段完整去畸变扫描到IMU系，待最终VIO融合位姿可用时以同一时间戳发布；原前端状态估计和显示话题保留。首次采集实测发现VIO时缓存被清空，已改为保留LIO扫描后发布。成功会话每帧约2.1万个点、10Hz，索引连续。
- `SessionWriter` 使用有界后台队列；队列溢出使记录失败，不默默丢帧。每帧文件/时间/位姿/点数与哈希校验；完整原始记录不被优化覆盖。
- 射线以真实LiDAR原点发出；IMU本体系点云用 `[0.032,0,0.232]` LiDAR外参，不误用世界原点或IMU到车体外参。全扫描先提供真实ray，再将车高端点用于occupied，地板不写黑；不把高位2D雷达的一条ray当成清除整个车高柱的依据。
- 三维体素面积投影到二维，避免只投中心漏掉格边低障碍。未确认hit保留unknown；弱free不覆盖已有强free。首次导航曾被弱free错误生成的unknown碎洞阻塞，已按实际失败修改投影规则并加入回归用例，随后复测成功。
- 本机MVS的旧libusb缺少PCL所需符号；仅在离线子进程预加载系统libusb，并提供工作区GTSAM/Metis库路径，未改变系统/MVS/手柄环境。

变更包 `mini_nav_fastlivo` 与 `mini_nav_bringup` 的构建、注册测试通过：22条测试结果（17个pytest用例加5个CTest包装记录），0失败/0跳过。前端 `fast_livo` 测试结果4条，0失败。实际二进制用例覆盖2cm/3cm分辨率的3cm低箱保留、地板不误黑、错位命中被后续ray撤销、弱free不制造未知洞。两套launch的 `--show-args` 检查通过。[测试日志](offline_wall_validation/colcon_test.log)。

2cm最终候选图已在独立224域重启，使用不同出生位置 `x=-1.75,y=-0.5,yaw=0.15`，设置初值后完成真实规划和短程Action到达：状态4、`goal_reached`、最终速度 `[0,0]`，地图哈希不变，单 `/map`、单 `/cmd_vel` 发布者，无建图节点。最小2D扫描距离约0.562m。此前3cm候选修正投影后也通过了一轮导航。[最终导航记录](offline_wall_validation/navigation/navigation_result.json)、[最终导航控制台](offline_wall_validation/navigation_resolution02.log)。

## 用户验收

加载环境后可独立启动最终图：

```bash
cd /home/a/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
source install_fastlivo/setup.bash
ros2 launch mini_nav_bringup fastlivo_navigation.launch.py \
  map_bundle:=/home/a/ros2_ws/maps/fastlivo2_offline/map_fbb572df42484633
```

默认出生点对应RViz地图原点附近。用 **2D Pose Estimate** 指定当前位置/朝向，定位有效后用 **2D Goal Pose**。观察墙厚时先只看 `/map`，随后分别开启规划/局部costmap，区分原始墙、硬安全圈和软代价圈。

要用PS5重新建一张图，退出导航后启动原 `fastlivo_mapping.launch.py`；默认记录开启，L1开关、摇杆和速度管理继续使用现有面板。停车并退出建图后，用 [包README](../../mini_nav_fastlivo/README.md) 的 `finalize_map` 命令从对应session生成新候选图。地图坐标、外参、三维定位图与二维图成对保存，导航仅加载旧图。

当前仅验证单层平地Gazebo；概率自由投影不是整车高可观测性证明。没有实现全局自动闭环，HBA未改善的原因未锁定；动态物体、实车、窄门和长期漂移仍待后续验收。若这张候选仍不满足墙边净空要求，现有完整会话支持继续测试其他位姿优化/TSDF后端，不需要回到不可重放的聚合PCD。
