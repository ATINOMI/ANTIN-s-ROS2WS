# 固定版本离线后端

`build.sh` 在工作区生成独立二进制，不向系统安装 ROS1。OctoMap 为本机系统 1.9 库；HBA 固定 `a0cdd474996fd9bb76888c8d1839b49b70aa0818`，GTSAM 4.2 固定 `4f66a491ffc83cf092d0d818b11dc35135521612`。运行数据和下载/构建/安装目录不加入 Git。

HBA 上游 GPLv2 许可随生成源码保留，CLI 是文件式实验程序；导航节点不链接 HBA/GTSAM。`prepare_hba.py` 只生成副本，移除 ROS1 发布与时间依赖，增加命令行参数、完整位姿读写与输入检查：保留分层 BA 和 PGO 数值路径，修正 EOF 多读一帧和四元数原地分量赋值问题。两层、最多500关键帧、35帧下限和4个内层线程是短场景实验约束。第一轮 HBA 输出没有改善墙厚，故它需要 `--backend hba` 显式启用。

默认 `probability` 使用原始融合位姿，重放全部完整去畸变 IMU 扫描。源点始终以真实 LiDAR 外参平移取 ray origin；只在碰撞高度范围内把端点作为 occupied。地面返回提供 free 且不写黑；每帧 hit/miss 去重，hit 优先。三维概率体素0.02m，二维0.05m；occupied投影按体素面积，未确认hit为unknown，孤立的弱free不阻挡已有强free。该模型面向现有平地场景，仍不是整车高可观测性证明。

本机环境存在海康 MVS 的旧 libusb，PCL IO 因缺少 `libusb_set_option` 无法直接加载。只对离线子进程预加载系统 libusb，并设置工作区 GTSAM/Metis 库路径；没有修改 MVS、系统库、用户手柄驱动或全局环境。

运行命令和边界见包 [README](../README.md)。所有地图都生成新目录；`occupancy.ot` 保留概率诊断数据，不把整个体素置信带或 TSDF 截断带作为实体墙。完整会话和逐帧哈希用于可重放性，未来可更换位姿优化后端；本轮没有实现全局闭环检测。
