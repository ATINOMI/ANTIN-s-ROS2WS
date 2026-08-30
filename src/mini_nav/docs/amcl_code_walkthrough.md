
# 官方 Nav2 AMCL 代码导读：从输入到定位输出

## 0. 目标和阅读方法

官方 AMCL 不是一个需要从头到尾硬啃的两千行文件。它是一个由 ROS 适配层、粒子滤波核心、地图计算、运动模型和激光模型共同组成的系统。

这份导读回答一个具体问题：

> 一次激光扫描到来后，官方 AMCL 如何把它变成新的机器人位姿和 map -> odom TF？

本文中的代码片段都来自项目内的官方 Jazzy 参考副本。片段只保留主线所需语句，省略版权头、日志和不影响承接关系的参数。完整上下文请打开对应原文件。

文中标注“来源”的代码块是从上游参考副本提取的关键片段；没有“来源”标注的 `text` 代码块是本文绘制的调用链、伪代码或概念示意，不是上游源码。

核心链路：

~~~
ROS 2 启动
    │
    ▼
AmclNode 生命周期配置
    │
    ├── 地图 OccupancyGrid ─► map_t + 距离场
    ├── 初始位姿 ───────────► 高斯粒子集合
    └── LaserScan + TF
            │
            ▼
       laserReceived()
            │
            ├── odom 增量 ─► 运动模型 ─► 粒子预测
            ├── 激光 + 地图 ─► 激光模型 ─► 粒子加权
            ├── 权重 ─► 重采样 / 随机恢复
            ├── 粒子 ─► 聚类 / 均值 / 协方差
            └── 输出 /amcl_pose 和 map -> odom
~~~

从概率模型看：

~~~
上一轮粒子集合
      │
      ├─ 里程计 u_t + 运动噪声 ─► 预测粒子集合
      │
      ├─ 激光 z_t + 地图 m ─────► 每个粒子的似然 / 权重
      │
      └─ 归一化 + 重采样 ───────► 下一轮粒子集合
~~~

## 1. 来源、版本和边界

官方参考快照：

- 目录：`[src/mini_nav/reference/nav2_amcl_jazzy/](../reference/nav2_amcl_jazzy/)`
- 上游包：`[nav2_amcl](https://github.com/ros-navigation/navigation2/tree/jazzy/nav2_amcl)`
- 分支：`jazzy`
- 快照提交：`f4108e5b1c2bce804a1aa0c7be6673a8eb4a1501`
- 包许可：`LGPL-2.1-or-later`；许可说明、版权头和 `LICENSE_NAVIGATION2` 均保留在参考副本中。

参考副本带有 `COLCON_IGNORE`，不会被本项目构建、安装或运行。当前官方 AMCL 运行入口仍使用系统安装版本；我们的自研算法继续放在 `mini_nav_core/localization/`。

阅读时先分层：

| 层 | 主要文件 | 责任 |
|---|---|---|
| ROS 适配层 | `amcl_node.hpp/.cpp`、`main.cpp` | 生命周期、参数、Topic、TF、服务、消息转换、总调度 |
| 粒子滤波层 | `pf/` | 粒子、权重、重采样、KLD、自适应统计 |
| 地图层 | `map/` | 栅格、坐标转换、距离场、射线 |
| 传感器层 | `sensors/laser/` | 将激光观测转成粒子权重 |
| 运动层 | `motion_model/` | 根据里程计增量扰动粒子 |

## 2. 进程启动：只创建节点，不直接定位

### 2.1 进程入口

来源：`[src/main.cpp](../reference/nav2_amcl_jazzy/src/main.cpp)`，函数 `main`。

~~~cpp
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<nav2_amcl::AmclNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}
~~~

这段代码只做三件事：

1. 初始化 ROS 2。
2. 创建 `AmclNode`。
3. 把节点交给 ROS 2 执行器。

它没有调用粒子滤波，也没有订阅激光。`AmclNode` 是生命周期节点，实际的 configure 和 activate 通常由 Nav2 的 lifecycle manager 触发。

### 2.2 节点声明生命周期接口

来源：`[include/nav2_amcl/amcl_node.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/amcl_node.hpp)`，类 `AmclNode`。

~~~cpp
class AmclNode : public nav2_util::LifecycleNode
{
protected:
  nav2_util::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & state) override;
  nav2_util::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & state) override;
~~~

生命周期的含义：

~~~
unconfigured --configure--> inactive --activate--> active
                                  ▲                  │
                                  └--deactivate-------┘
~~~

传感器可能一直在发消息，但 AMCL 只有在 `active_ == true` 时才真正处理激光。这避免了节点尚未准备好地图、TF、发布器时就使用传感器数据。

### 2.3 CMake 把 AMCL 装配成多个库

来源：`[CMakeLists.txt](../reference/nav2_amcl_jazzy/CMakeLists.txt)`，子目录装配和链接部分。

~~~cmake
add_subdirectory(src/pf)
add_subdirectory(src/map)
add_subdirectory(src/motion_model)
add_subdirectory(src/sensors)

add_library(amcl_core SHARED
  src/amcl_node.cpp
)

target_link_libraries(amcl_core
  map_lib pf_lib sensors_lib
)
~~~

因此 `amcl_node.cpp` 是总调度器，不是完整算法。粒子滤波、地图和激光模型分别编译成库，再被 ROS 节点链接进来。

## 3. 配置阶段：按依赖顺序组装对象

### 3.1 参数是算法的运行配置

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，构造函数 `AmclNode::AmclNode`。

~~~cpp
add_parameter("base_frame_id",
  rclcpp::ParameterValue(std::string("base_footprint")));
add_parameter("global_frame_id",
  rclcpp::ParameterValue(std::string("map")));
add_parameter("odom_frame_id",
  rclcpp::ParameterValue(std::string("odom")));

add_parameter("max_particles", rclcpp::ParameterValue(2000));
add_parameter("min_particles", rclcpp::ParameterValue(500));
add_parameter("max_beams", rclcpp::ParameterValue(60));
add_parameter("laser_model_type",
  rclcpp::ParameterValue(std::string("likelihood_field")));
add_parameter("update_min_d", rclcpp::ParameterValue(0.25));
add_parameter("update_min_a", rclcpp::ParameterValue(0.2));
~~~

这些参数分别控制：

- 坐标系：`map`、`odom`、`base_footprint`。
- 粒子数量上下限。
- 每次激光更新使用的 beam 数量。
- 激光观测模型。
- 机器人移动多少后才更新滤波器。

构造函数主要声明参数；真正读取参数和创建运行对象在 `on_configure` 中完成。

### 3.2 `on_configure` 是组装总入口

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `AmclNode::on_configure`。

~~~cpp
callback_group_ = create_callback_group(
  rclcpp::CallbackGroupType::MutuallyExclusive, false);

initParameters();
initTransforms();
initParticleFilter();
initLaserScan();
initMessageFilters();
initPubSub();
initServices();
initOdometry();
~~~

它的承接关系是：

~~~
参数
  ├─► TF
  ├─► 粒子滤波器
  ├─► 激光状态
  ├─► MessageFilter
  ├─► 发布器和订阅器
  ├─► 服务
  └─► 运动模型
~~~

`initMessageFilters` 依赖已经存在的 `tf_buffer_`，所以必须在 `initTransforms` 之后。`initOdometry` 需要 alpha 噪声参数，并加载运动模型插件。

### 3.3 `on_activate` 才允许处理传感器

来源：同一文件，函数 `AmclNode::on_activate`。

~~~cpp
pose_pub_->on_activate();
particle_cloud_pub_->on_activate();

first_pose_sent_ = false;
active_ = true;

if (set_initial_pose_) {
  auto msg =
    std::make_shared<geometry_msgs::msg::PoseWithCovarianceStamped>();
  msg->header.stamp = now();
  msg->header.frame_id = global_frame_id_;
  msg->pose.pose.position.x = initial_pose_x_;
  msg->pose.pose.position.y = initial_pose_y_;
  msg->pose.pose.orientation =
    orientationAroundZAxis(initial_pose_yaw_);

  initialPoseReceived(msg);
}
~~~

生命周期发布器必须显式激活。配置参数中的初始位姿被包装成和 `/initialpose` 相同的消息，然后复用同一条初始位姿逻辑。

## 4. 先认识算法数据结构

### 4.1 一个粒子就是 pose + weight

来源：`[include/nav2_amcl/pf/pf.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/pf/pf.hpp)` 和 `[include/nav2_amcl/pf/pf_vector.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/pf/pf_vector.hpp)`。

~~~cpp
typedef struct
{
  double v[3];
} pf_vector_t;

typedef struct
{
  pf_vector_t pose;
  double weight;
} pf_sample_t;
~~~

三个 pose 分量约定为：

~~~
pose.v[0] = x
pose.v[1] = y
pose.v[2] = yaw
~~~

这是二维定位状态，不包含 z、roll、pitch。

### 4.2 粒子集合保存双缓冲、KD 树和统计量

来源：`pf.hpp`，`pf_sample_set_t` 和 `pf_t`。

~~~cpp
typedef struct _pf_sample_set_t
{
  int sample_count;
  pf_sample_t * samples;

  pf_kdtree_t * kdtree;

  int cluster_count, cluster_max_count;
  pf_cluster_t * clusters;

  pf_vector_t mean;
  pf_matrix_t cov;
  int converged;
} pf_sample_set_t;

typedef struct _pf_t
{
  int min_samples, max_samples;
  double pop_err, pop_z;

  int current_set;
  pf_sample_set_t sets[2];

  double w_slow, w_fast;
  double alpha_slow, alpha_fast;
  pf_init_model_fn_t random_pose_fn;
} pf_t;
~~~

几个关键设计：

1. `sets[2]`：从当前集合读，向另一个集合写，重采样完成后交换 `current_set`。
2. `kdtree`：把粒子分桶，用于聚类和 KLD 自适应采样。
3. `w_slow`、`w_fast`：记录长期和短期观测质量，用于定位恢复。

### 4.3 函数指针是算法模块之间的接缝

来源：`pf.hpp`。

~~~cpp
typedef pf_vector_t (* pf_init_model_fn_t) (void * init_data);

typedef double (* pf_sensor_model_fn_t) (
  void * sensor_data,
  struct _pf_sample_set_t * set);

pf_t * pf_alloc(
  int min_samples, int max_samples,
  double alpha_slow, double alpha_fast,
  pf_init_model_fn_t random_pose_fn);

void pf_init(pf_t * pf, pf_vector_t mean, pf_matrix_t cov);
void pf_init_model(pf_t * pf,
  pf_init_model_fn_t init_fn, void * init_data);
void pf_update_sensor(pf_t * pf,
  pf_sensor_model_fn_t sensor_fn, void * sensor_data);
void pf_update_resample(pf_t * pf, void * random_pose_data);
~~~

承接关系：

~~~
AMCL 节点提供初始化函数和地图
    └─► pf_init / pf_init_model

Laser 模型提供 sensorFunction
    └─► pf_update_sensor

AMCL 节点控制重采样时机
    └─► pf_update_resample
~~~

粒子滤波器不需要知道 ROS 消息，也不需要包含 `LaserScan`。

## 5. 地图：从 OccupancyGrid 到算法地图

### 5.1 算法地图的结构

来源：`[include/nav2_amcl/map/map.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/map/map.hpp)`。

~~~cpp
typedef struct
{
  int8_t occ_state;  // -1 free, 0 unknown, +1 occupied
  float occ_dist;    // distance to nearest occupied cell
} map_cell_t;

typedef struct
{
  double origin_x, origin_y;
  double scale;
  int size_x, size_y;
  map_cell_t * cells;
  double max_occ_dist;
} map_t;
~~~

ROS 的 `OccupancyGrid` 是通信消息，`map_t` 是算法数据。`occ_dist` 是为 likelihood-field 模型预计算的“该栅格到最近障碍物的距离”。

同一文件中的宏完成世界坐标和栅格坐标转换：

~~~cpp
#define MAP_WXGX(map, i) \
  (map->origin_x + ((i) - map->size_x / 2) * map->scale)

#define MAP_GXWX(map, x) \
  (floor((x - map->origin_x) / map->scale + 0.5) + map->size_x / 2)

#define MAP_VALID(map, i, j) \
  ((i >= 0) && (i < map->size_x) && \
   (j >= 0) && (j < map->size_y))
~~~

激光模型后面会用 `MAP_GXWX` 把激光端点映射到地图栅格。

### 5.2 地图消息入口

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `mapReceived` 和 `handleMapMessage`。

~~~cpp
void AmclNode::mapReceived(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  if (!nav2_util::validateMsg(*msg)) {
    RCLCPP_ERROR(get_logger(),
      "Received map message is malformed. Rejecting.");
    return;
  }
  if (first_map_only_ && first_map_received_) {
    return;
  }

  handleMapMessage(*msg);
  first_map_received_ = true;
}

void AmclNode::handleMapMessage(
  const nav_msgs::msg::OccupancyGrid & msg)
{
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  freeMapDependentMemory();
  map_ = convertMap(msg);
}
~~~

地图替换时先清理旧地图和依赖旧地图的激光对象，再生成新地图。否则激光模型可能继续持有已经失效的地图指针。

### 5.3 `convertMap` 改变栅格语义

来源：同一文件，函数 `AmclNode::convertMap`。

~~~cpp
map_t * map = map_alloc();

map->size_x = map_msg.info.width;
map->size_y = map_msg.info.height;
map->scale = map_msg.info.resolution;
map->origin_x = map_msg.info.origin.position.x +
  (map->size_x / 2) * map->scale;
map->origin_y = map_msg.info.origin.position.y +
  (map->size_y / 2) * map->scale;

map->cells = reinterpret_cast<map_cell_t *>(
  malloc(sizeof(map_cell_t) * map->size_x * map->size_y));

for (int i = 0; i < map->size_x * map->size_y; i++) {
  if (map_msg.data[i] == 0) {
    map->cells[i].occ_state = -1;
  } else if (map_msg.data[i] == 100) {
    map->cells[i].occ_state = +1;
  } else {
    map->cells[i].occ_state = 0;
  }
}
~~~

转换规则：

| ROS 占用值 | AMCL 内部 |
|---:|---:|
| `0` | `-1`，自由 |
| `100` | `+1`，占用 |
| 其他值 | `0`，未知 |

### 5.4 为激光模型建立距离场

来源：`[src/map/map_cspace.cpp](../reference/nav2_amcl_jazzy/src/map/map_cspace.cpp)`，函数 `map_update_cspace`。

~~~cpp
map->max_occ_dist = max_occ_dist;

for (int i = 0; i < map->size_x; i++) {
  for (int j = 0; j < map->size_y; j++) {
    if (map->cells[MAP_INDEX(map, i, j)].occ_state == +1) {
      map->cells[MAP_INDEX(map, i, j)].occ_dist = 0.0;
      Q.push(cell);
    } else {
      map->cells[MAP_INDEX(map, i, j)].occ_dist = max_occ_dist;
    }
  }
}

while (!Q.empty()) {
  CellData current_cell = Q.top();
  enqueue(/* 邻居栅格和障碍源 */, Q, cdm, marked);
  Q.pop();
}
~~~

它从障碍栅格开始，把距离向邻居传播。完成后，激光端点只需查询一个栅格就能获得最近障碍距离。

## 6. 初始位姿：把粒子“放进地图”

### 6.1 初始位姿先检查 frame 和生命周期

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `initialPoseReceived`。

~~~cpp
if (!nav2_util::validateMsg(*msg)) {
  return;
}

if (nav2_util::strip_leading_slash(msg->header.frame_id)
    != global_frame_id_) {
  return;
}

last_published_pose_ = *msg;

if (!active_) {
  init_pose_received_on_inactive = true;
  return;
}

handleInitialPose(*msg);
~~~

初始位姿通常来自 RViz 的 `/initialpose`，要求表达在全局地图坐标系 `map` 中。节点未激活时先保存，不立即初始化。

### 6.2 初始位姿变成均值和协方差

来源：同一文件，函数 `handleInitialPose`。

~~~cpp
pf_vector_t pf_init_pose_mean = pf_vector_zero();
pf_init_pose_mean.v[0] = pose_new.getOrigin().x();
pf_init_pose_mean.v[1] = pose_new.getOrigin().y();
pf_init_pose_mean.v[2] =
  tf2::getYaw(pose_new.getRotation());

pf_matrix_t pf_init_pose_cov = pf_matrix_zero();
for (int i = 0; i < 2; i++) {
  for (int j = 0; j < 2; j++) {
    pf_init_pose_cov.m[i][j] =
      msg.pose.covariance[6 * i + j];
  }
}
pf_init_pose_cov.m[2][2] =
  msg.pose.covariance[6 * 5 + 5];

pf_init(pf_, pf_init_pose_mean, pf_init_pose_cov);
pf_init_ = false;
initial_pose_is_known_ = true;
~~~

这不是把所有粒子放在同一个点，而是：

~~~
RViz pose + covariance
        │
        ▼
3 维均值 [x, y, yaw] + 3×3 协方差
        │
        ▼
从高斯分布采样一批粒子
~~~

在调用 `pf_init` 前，AMCL 还会根据消息时间戳查询 TF，把初始 pose 和当前 odom 关系对齐。

### 6.3 `pf_init` 从高斯分布采样

来源：`[src/pf/pf.c](../reference/nav2_amcl_jazzy/src/pf/pf.c)`，函数 `pf_init`。

~~~c
set = pf->sets + pf->current_set;
pf_kdtree_clear(set->kdtree);
set->sample_count = pf->max_samples;

pdf = pf_pdf_gaussian_alloc(mean, cov);

for (i = 0; i < set->sample_count; i++) {
  sample = set->samples + i;
  sample->weight = 1.0 / pf->max_samples;
  sample->pose = pf_pdf_gaussian_sample(pdf);
  pf_kdtree_insert(set->kdtree,
    sample->pose, sample->weight);
}

pf_cluster_stats(pf, set);
pf_init_converged(pf);
~~~

粒子初始时权重相同。此时 AMCL 只有一个带不确定性的候选分布，还没有用激光判断哪个候选更好。

### 6.4 全局定位使用自由空间随机采样

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `uniformPoseGenerator`。

~~~cpp
unsigned int rand_index =
  drand48() * free_space_indices.size();
AmclNode::Point2D free_point =
  free_space_indices[rand_index];

pf_vector_t p;
p.v[0] = MAP_WXGX(map, free_point.x);
p.v[1] = MAP_WYGY(map, free_point.y);
p.v[2] = drand48() * 2 * M_PI - M_PI;
return p;
~~~

全局重定位时，位置来自地图自由栅格，yaw 随机。服务 `reinitialize_global_localization` 最终调用：

~~~cpp
pf_init_model(
  pf_,
  (pf_init_model_fn_t)AmclNode::uniformPoseGenerator,
  reinterpret_cast<void *>(map_));
~~~

同一个粒子滤波器接口既支持“围绕初始位姿采样”，也支持“在地图自由空间采样”。

## 7. TF 和激光订阅：算法更新的前置条件

### 7.1 地图使用 Transient Local QoS

来源：`amcl_node.cpp`，函数 `initPubSub`。

~~~cpp
map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
  map_topic_,
  rclcpp::QoS(rclcpp::KeepLast(1))
    .transient_local().reliable(),
  std::bind(&AmclNode::mapReceived, this, _1));
~~~

地图是静态或低频数据，`TRANSIENT_LOCAL` 允许晚加入的订阅者收到发布者保存的最后一张地图。

### 7.2 激光先经过 TF MessageFilter

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `initMessageFilters`。

~~~cpp
laser_scan_sub_ = std::make_unique<
  message_filters::Subscriber<sensor_msgs::msg::LaserScan,
  rclcpp_lifecycle::LifecycleNode>>(
    shared_from_this(), scan_topic_,
    rmw_qos_profile_sensor_data, sub_opt);

laser_scan_filter_ = std::make_unique<
  tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
    *laser_scan_sub_, *tf_buffer_, odom_frame_id_, 10,
    get_node_logging_interface(),
    get_node_clock_interface(),
    transform_tolerance_);

laser_scan_connection_ =
  laser_scan_filter_->registerCallback(
    std::bind(&AmclNode::laserReceived, this, _1));
~~~

它不是普通的“收到就调用”。它会等待对应时间的 TF 可用，至少涉及：

~~~
laser frame -> base frame
base frame  -> odom frame
~~~

如果 TF 时间不匹配，就不会把这一帧交给算法。

### 7.3 第一次遇到激光 frame 时创建模型

来源：`amcl_node.cpp`，函数 `addNewScanner`。

~~~cpp
lasers_.push_back(createLaserObject());
lasers_update_.push_back(true);
laser_index = frame_to_laser_.size();

tf_buffer_->transform(
  ident, laser_pose, base_frame_id_,
  transform_tolerance_);

pf_vector_t laser_pose_v;
laser_pose_v.v[0] = laser_pose.pose.position.x;
laser_pose_v.v[1] = laser_pose.pose.position.y;
laser_pose_v.v[2] = 0;

lasers_[laser_index]->SetLaserPose(laser_pose_v);
frame_to_laser_[laser_scan->header.frame_id] =
  laser_index;
~~~

AMCL 保存激光相对机器人 base 的安装位姿。传感器不一定在机器人中心，端点投影时必须考虑这段外参。

## 8. 一帧激光进入主调度器

### 8.1 回调先做门控

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `laserReceived`。

~~~cpp
std::lock_guard<std::recursive_mutex> lock(mutex_);

if (!active_) {
  return;
}
if (!first_map_received_) {
  RCLCPP_WARN(get_logger(), "Waiting for map....");
  return;
}
~~~

因此一帧激光要进入算法，必须满足：

~~~
节点 active
地图已收到
MessageFilter 已确认 TF 可用
~~~

### 8.2 获取扫描时刻的 odom pose

来源：同一文件，函数 `laserReceived` 和 `getOdomPose`。

~~~cpp
pf_vector_t pose;
if (!getOdomPose(
    latest_odom_pose_,
    pose.v[0], pose.v[1], pose.v[2],
    laser_scan->header.stamp,
    base_frame_id_))
{
  RCLCPP_ERROR(get_logger(),
    "Couldn't determine robot's pose associated "
    "with laser scan");
  return;
}
~~~

`getOdomPose` 的核心：

~~~cpp
geometry_msgs::msg::PoseStamped ident;
ident.header.frame_id =
  nav2_util::strip_leading_slash(frame_id);
ident.header.stamp = sensor_timestamp;
tf2::toMsg(
  tf2::Transform::getIdentity(), ident.pose);

tf_buffer_->transform(
  ident, odom_pose, odom_frame_id_);

x = odom_pose.pose.position.x;
y = odom_pose.pose.position.y;
yaw = tf2::getYaw(
  odom_pose.pose.orientation);
~~~

这里的 pose 是 odom 坐标系下的机器人位姿，用于计算运动增量；它不是最终发布的地图坐标系 pose。

### 8.3 首帧建立 odom 基准

来源：`laserReceived`。

~~~cpp
pf_vector_t delta = pf_vector_zero();

if (!pf_init_) {
  pf_odom_pose_ = pose;
  pf_init_ = true;

  for (unsigned int i = 0;
    i < lasers_update_.size(); i++) {
    lasers_update_[i] = true;
  }

  force_publication = true;
  resample_count_ = 0;
} else {
  if (shouldUpdateFilter(pose, delta)) {
    for (unsigned int i = 0;
      i < lasers_update_.size(); i++) {
      lasers_update_[i] = true;
    }
  }
}
~~~

注意：

~~~
pf_      = 粒子滤波器对象
pf_init_ = 是否已经建立第一轮 odom 更新基准
~~~

`pf_` 在配置阶段就创建；`pf_init_` 直到激光回调拿到第一份 odom pose 后才变为 true。

## 9. 运动更新：让每个粒子根据 odom 预测

### 9.1 是否达到运动阈值

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `shouldUpdateFilter`。

~~~cpp
delta.v[0] =
  pose.v[0] - pf_odom_pose_.v[0];
delta.v[1] =
  pose.v[1] - pf_odom_pose_.v[1];
delta.v[2] = angleutils::angle_diff(
  pose.v[2], pf_odom_pose_.v[2]);

bool update =
  fabs(delta.v[0]) > d_thresh_ ||
  fabs(delta.v[1]) > d_thresh_ ||
  fabs(delta.v[2]) > a_thresh_;

update = update || force_update_;
return update;
~~~

这一步减少不必要的计算。机器人平移超过 `d_thresh_`、旋转超过 `a_thresh_`，或收到无运动更新请求时，才标记需要更新。

### 9.2 差分模型把增量分成 rot1、trans、rot2

来源：`[src/motion_model/differential_motion_model.cpp](../reference/nav2_amcl_jazzy/src/motion_model/differential_motion_model.cpp)`，函数 `DifferentialMotionModel::odometryUpdate`。

~~~cpp
pf_sample_set_t * set =
  pf->sets + pf->current_set;
pf_vector_t old_pose =
  pf_vector_sub(pose, delta);

double delta_rot1;
double delta_trans;
double delta_rot2;

delta_rot1 = angleutils::angle_diff(
  atan2(delta.v[1], delta.v[0]),
  old_pose.v[2]);
delta_trans = sqrt(
  delta.v[0] * delta.v[0] +
  delta.v[1] * delta.v[1]);
delta_rot2 = angleutils::angle_diff(
  delta.v[2], delta_rot1);
~~~

一次运动被理解为：

~~~
先旋转 rot1
再平移 trans
最后旋转 rot2
~~~

### 9.3 对每个粒子分别加入运动噪声

来源：同一文件，函数 `DifferentialMotionModel::odometryUpdate`。

~~~cpp
for (int i = 0; i < set->sample_count; i++) {
  pf_sample_t * sample = set->samples + i;

  delta_rot1_hat = angleutils::angle_diff(
    delta_rot1,
    pf_ran_gaussian(sqrt(
      alpha1_ * delta_rot1_noise * delta_rot1_noise +
      alpha2_ * delta_trans * delta_trans)));

  delta_trans_hat = delta_trans - pf_ran_gaussian(sqrt(
    alpha3_ * delta_trans * delta_trans +
    alpha4_ * delta_rot1_noise * delta_rot1_noise +
    alpha4_ * delta_rot2_noise * delta_rot2_noise));

  delta_rot2_hat = angleutils::angle_diff(
    delta_rot2,
    pf_ran_gaussian(sqrt(
      alpha1_ * delta_rot2_noise * delta_rot2_noise +
      alpha2_ * delta_trans * delta_trans)));

  sample->pose.v[0] += delta_trans_hat *
    cos(sample->pose.v[2] + delta_rot1_hat);
  sample->pose.v[1] += delta_trans_hat *
    sin(sample->pose.v[2] + delta_rot1_hat);
  sample->pose.v[2] +=
    delta_rot1_hat + delta_rot2_hat;
}
~~~

同一份 odom 增量会给不同粒子产生不同结果。概率含义是：

~~~
p(x_t | x_{t-1}, u_t)
~~~

这一步只看旧粒子和 odom，不看地图和激光。

## 10. 激光预处理：从 ROS 消息到算法数据

### 10.1 `LaserData` 是算法看到的输入

来源：`[include/nav2_amcl/sensors/laser/laser.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/sensors/laser/laser.hpp)`。

~~~cpp
class LaserData
{
public:
  Laser * laser;
  int range_count;
  double range_max;
  double(*ranges)[2];
};
~~~

每个观测最终表达成：

~~~
ranges[i][0] = range
ranges[i][1] = bearing
~~~

### 10.2 `updateFilter` 完成消息转换

来源：`amcl_node.cpp`，函数 `updateFilter`。

~~~cpp
nav2_amcl::LaserData ldata;
ldata.laser = lasers_[laser_index].get();
ldata.range_count = laser_scan->ranges.size();

double angle_min = tf2::getYaw(min_q.quaternion);
double angle_increment =
  tf2::getYaw(inc_q.quaternion) - angle_min;

ldata.ranges =
  new double[ldata.range_count][2];

for (int i = 0; i < ldata.range_count; i++) {
  ldata.ranges[i][0] =
    laser_scan->ranges[i];
  ldata.ranges[i][1] =
    angle_min + i * angle_increment;
}

lasers_[laser_index]->sensorUpdate(
  pf_,
  reinterpret_cast<nav2_amcl::LaserData *>(&ldata));
~~~

ROS 适配层在这里完成：

- 激光角度变换。
- 最小、最大量程处理。
- `LaserScan` 到 `LaserData` 的转换。
- 将数据交给具体激光模型。

### 10.3 通过抽象接口选择激光模型

来源：`amcl_node.cpp`，函数 `createLaserObject`；以及 `laser.hpp`。

~~~cpp
class Laser
{
public:
  virtual bool sensorUpdate(
    pf_t * pf, LaserData * data) = 0;
};
~~~

模型选择：

~~~cpp
if (sensor_model_type_ == "beam") {
  return std::make_unique<nav2_amcl::BeamModel>(
    /* 参数 */, map_);
}

if (sensor_model_type_ == "likelihood_field_prob") {
  return std::make_unique<
    nav2_amcl::LikelihoodFieldModelProb>(
      /* 参数 */, map_);
}

return std::make_unique<
  nav2_amcl::LikelihoodFieldModel>(
    /* 参数 */, map_);
~~~

默认通常是 `likelihood_field`，所以应先读 `likelihood_field_model.cpp`。

## 11. 观测更新：激光如何给粒子打分

### 11.1 Likelihood Field 的核心

来源：`[src/sensors/laser/likelihood_field_model.cpp](../reference/nav2_amcl_jazzy/src/sensors/laser/likelihood_field_model.cpp)`，函数 `LikelihoodFieldModel::sensorFunction`。

~~~cpp
for (j = 0; j < set->sample_count; j++) {
  sample = set->samples + j;
  pose = sample->pose;

  pose = pf_vector_coord_add(
    self->laser_pose_, pose);
  p = 1.0;

  for (i = 0; i < data->range_count; i += step) {
    obs_range = data->ranges[i][0];
    obs_bearing = data->ranges[i][1];

    if (obs_range >= data->range_max ||
      obs_range != obs_range) {
      continue;
    }

    hit.v[0] = pose.v[0] +
      obs_range * cos(pose.v[2] + obs_bearing);
    hit.v[1] = pose.v[1] +
      obs_range * sin(pose.v[2] + obs_bearing);

    int mi = MAP_GXWX(self->map_, hit.v[0]);
    int mj = MAP_GYWY(self->map_, hit.v[1]);
~~~

对每个粒子、每条 beam：

1. 把粒子 pose 和激光安装 pose 合成。
2. 使用实际 range 和 bearing 计算激光端点。
3. 把端点转成地图栅格。
4. 查询这个端点离最近障碍物有多远。

### 11.2 查询 `occ_dist` 并形成似然

来源：同一文件，函数 `LikelihoodFieldModel::sensorFunction`。

~~~cpp
if (!MAP_VALID(self->map_, mi, mj)) {
  z = self->map_->max_occ_dist;
} else {
  z = self->map_->cells[
    MAP_INDEX(self->map_, mi, mj)].occ_dist;
}

double z_hit_denom =
  2 * self->sigma_hit_ * self->sigma_hit_;
double z_rand_mult =
  1.0 / data->range_max;

pz = 0.0;
pz += self->z_hit_ *
  exp(-(z * z) / z_hit_denom);
pz += self->z_rand_ * z_rand_mult;

p += pz * pz * pz;
~~~

直觉：

~~~
端点离障碍越近
    └─► z 越小 ─► pz 越大 ─► 粒子更可信

端点离障碍越远
    └─► z 越大 ─► pz 越小 ─► 粒子更不可信
~~~

当前实现使用 `p += pz * pz * pz` 的经验组合方式；不要只根据模型名字猜公式，要以源码的实际路径为准。

### 11.3 具体模型把工作交给通用滤波器

来源：`likelihood_field_model.cpp`，函数 `sensorUpdate`。

~~~cpp
bool LikelihoodFieldModel::sensorUpdate(
  pf_t * pf, LaserData * data)
{
  if (max_beams_ < 2) {
    return false;
  }

  pf_update_sensor(
    pf,
    (pf_sensor_model_fn_t)sensorFunction,
    data);
  return true;
}
~~~

所以模块承接是：

~~~
LikelihoodFieldModel::sensorFunction
    └─ 修改每个 sample->weight
          │
          ▼
pf_update_sensor
    └─ 负责归一化和恢复统计
~~~

### 11.4 Beam Model 是另一条路径

来源：`[src/sensors/laser/beam_model.cpp](../reference/nav2_amcl_jazzy/src/sensors/laser/beam_model.cpp)` 和 `[src/map/map_range.c](../reference/nav2_amcl_jazzy/src/map/map_range.c)`。

两者区别：

~~~
Likelihood Field：
    实际测量端点
        └─► 查询端点到最近障碍的距离

Beam Model：
    从粒子 pose 沿激光方向射线追踪
        └─► 得到地图预测距离
        └─► 和实际测量比较
~~~

`map_calc_range` 使用 Bresenham 逐栅格追踪射线，主要服务于 Beam Model。默认 likelihood-field 路径不会对每条 beam 做完整射线预测。

## 12. 权重归一化：通用粒子滤波器接管

来源：`[src/pf/pf.c](../reference/nav2_amcl_jazzy/src/pf/pf.c)`，函数 `pf_update_sensor`。

~~~c
set = pf->sets + pf->current_set;

total = (*sensor_fn)(sensor_data, set);

if (total > 0.0) {
  double w_avg = 0.0;

  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    w_avg += sample->weight;
    sample->weight /= total;
  }

  w_avg /= set->sample_count;
  pf->w_slow +=
    pf->alpha_slow * (w_avg - pf->w_slow);
  pf->w_fast +=
    pf->alpha_fast * (w_avg - pf->w_fast);
}
~~~

输入输出：

~~~
sensorFunction
    └─ 返回所有粒子的总权重 total
          │
          ▼
pf_update_sensor
    ├─ 每个权重除以 total
    ├─ 得到权重和为 1 的分布
    └─ 更新 w_slow / w_fast
~~~

如果所有粒子的总权重为零，代码会退回均匀分布：

~~~c
else {
  for (i = 0; i < set->sample_count; i++) {
    sample = set->samples + i;
    sample->weight = 1.0 / set->sample_count;
  }
}
~~~

这是一个防御性分支，避免传感器异常时产生全零或不可归一化的分布。

## 13. 重采样：高权重粒子留下来

### 13.1 使用两个集合和累计概率表

来源：`pf.c`，函数 `pf_update_resample`。

~~~c
set_a = pf->sets + pf->current_set;
set_b = pf->sets +
  (pf->current_set + 1) % 2;

c = (double *)malloc(
  sizeof(double) * (set_a->sample_count + 1));
c[0] = 0.0;

for (i = 0; i < set_a->sample_count; i++) {
  c[i + 1] =
    c[i] + set_a->samples[i].weight;
}

pf_kdtree_clear(set_b->kdtree);
set_b->sample_count = 0;
~~~

例如权重：

~~~
[0.1, 0.2, 0.7]
~~~

累计概率：

~~~
[0.0, 0.1, 0.3, 1.0]
~~~

随机数落在哪个区间，就抽到哪个粒子。权重越大的粒子，区间越长，越容易被复制。

### 13.2 普通复制和随机恢复共存

来源：同一函数 `pf_update_resample`。

~~~c
w_diff = 1.0 -
  pf->w_fast / pf->w_slow;
if (w_diff < 0.0) {
  w_diff = 0.0;
}

while (set_b->sample_count < pf->max_samples) {
  sample_b =
    set_b->samples + set_b->sample_count++;

  if (drand48() < w_diff) {
    sample_b->pose =
      (pf->random_pose_fn)(random_pose_data);
  } else {
    double r = drand48();

    for (i = 0; i < set_a->sample_count; i++) {
      if ((c[i] <= r) &&
        (r < c[i + 1])) {
        break;
      }
    }

    sample_b->pose =
      set_a->samples[i].pose;
  }

  sample_b->weight = 1.0;
  pf_kdtree_insert(
    set_b->kdtree,
    sample_b->pose,
    sample_b->weight);
}
~~~

当短期观测质量明显差于长期质量时，`w_diff` 增大，部分新粒子直接从地图自由空间生成。这是 AMCL 从错误位置恢复的重要机制。

### 13.3 KLD 自适应采样决定需要多少粒子

来源：`pf.c`，函数 `pf_resample_limit`。

~~~c
if (k <= 1) {
  return pf->max_samples;
}

b = 2 / (9 * ((double) k - 1));
c = sqrt(2 / (9 * ((double) k - 1))) *
  pf->pop_z;
x = 1 - b + c;

n = (int)ceil(
  (k - 1) / (2 * pf->pop_err) *
  x * x * x);

if (n < pf->min_samples) {
  return pf->min_samples;
}
if (n > pf->max_samples) {
  return pf->max_samples;
}
return n;
~~~

`k` 是 KD 树中的有效空间桶数量：

- 分布集中，`k` 小，需要的粒子可以少。
- 分布分散，`k` 大，需要更多粒子表达多峰分布。
- 最终粒子数仍限制在 `min_samples` 和 `max_samples` 之间。

### 13.4 重采样结束后交换集合并重新统计

来源：同一函数 `pf_update_resample`。

~~~c
for (i = 0; i < set_b->sample_count; i++) {
  sample_b = set_b->samples + i;
  sample_b->weight /= total;
}

pf_cluster_stats(pf, set_b);
pf->current_set =
  (pf->current_set + 1) % 2;
pf_update_converged(pf);

free(c);
~~~

直到交换 `current_set` 后，新集合才成为下一轮当前集合。

## 14. 统计：从粒子云提取当前假设

### 14.1 KD 树聚类

来源：`[src/pf/pf_kdtree.c](../reference/nav2_amcl_jazzy/src/pf/pf_kdtree.c)`，函数 `pf_kdtree_cluster`。

~~~c
for (i = 0; i < self->node_count; i++) {
  node = self->nodes + i;
  if (node->leaf) {
    node->cluster = -1;
    queue[queue_count++] = node;
  }
}

while (queue_count > 0) {
  node = queue[--queue_count];

  if (node->cluster >= 0) {
    continue;
  }

  node->cluster = cluster_count++;
  pf_kdtree_cluster_node(
    self, node, 0);
}
~~~

相邻空间桶被划分为同一粒子簇。这样面对重复房间、对称走廊等情况时，AMCL 能保留多个定位假设，而不是立刻把多峰分布平均成一个错误位置。

### 14.2 计算位置均值和圆周 yaw 均值

来源：`[src/pf/pf.c](../reference/nav2_amcl_jazzy/src/pf/pf.c)`，函数 `pf_cluster_stats`。

~~~c
cluster->m[0] +=
  sample->weight * sample->pose.v[0];
cluster->m[1] +=
  sample->weight * sample->pose.v[1];
cluster->m[2] +=
  sample->weight * cos(sample->pose.v[2]);
cluster->m[3] +=
  sample->weight * sin(sample->pose.v[2]);

cluster->mean.v[0] =
  cluster->m[0] / cluster->weight;
cluster->mean.v[1] =
  cluster->m[1] / cluster->weight;
cluster->mean.v[2] =
  atan2(cluster->m[3], cluster->m[2]);
~~~

yaw 不能直接进行普通算术平均。例如 `179°` 和 `-179°` 的正确平均结果应该接近 `180°`，而不是 `0°`。因此官方先平均 sin/cos，再用 `atan2` 还原角度。

### 14.3 选择最大权重簇

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `getMaxWeightHyp`。

~~~cpp
hyps.resize(
  pf_->sets[pf_->current_set].cluster_count);

for (int hyp_count = 0;
  hyp_count < pf_->sets[pf_->current_set].cluster_count;
  hyp_count++)
{
  double weight;
  pf_vector_t pose_mean;
  pf_matrix_t pose_cov;

  if (!pf_get_cluster_stats(
      pf_, hyp_count, &weight,
      &pose_mean, &pose_cov)) {
    return false;
  }

  hyps[hyp_count].weight = weight;
  hyps[hyp_count].pf_pose_mean = pose_mean;
  hyps[hyp_count].pf_pose_cov = pose_cov;

  if (hyps[hyp_count].weight > max_weight) {
    max_weight = hyps[hyp_count].weight;
    max_weight_hyp = hyp_count;
  }
}
~~~

AMCL 输出的 pose 不是随机挑一个粒子，也不是无条件对所有粒子求平均，而是最高权重空间簇的统计均值。

## 15. 输出：/amcl_pose、粒子云和 map -> odom

### 15.1 发布粒子云

来源：`amcl_node.cpp`，函数 `publishParticleCloud`。

~~~cpp
auto cloud_with_weights_msg =
  std::make_unique<nav2_msgs::msg::ParticleCloud>();

cloud_with_weights_msg->header.stamp = now();
cloud_with_weights_msg->header.frame_id =
  global_frame_id_;
cloud_with_weights_msg->particles.resize(
  set->sample_count);

for (int i = 0; i < set->sample_count; i++) {
  cloud_with_weights_msg->particles[i].pose.position.x =
    set->samples[i].pose.v[0];
  cloud_with_weights_msg->particles[i].pose.position.y =
    set->samples[i].pose.v[1];
  cloud_with_weights_msg->particles[i].weight =
    set->samples[i].weight;
}

particle_cloud_pub_->publish(
  std::move(cloud_with_weights_msg));
~~~

粒子云位于 `map` 坐标系，主要用于 RViz 可视化和调试。

### 15.2 发布 `/amcl_pose`

来源：`amcl_node.cpp`，函数 `publishAmclPose`。

~~~cpp
auto p = std::make_unique<
  geometry_msgs::msg::PoseWithCovarianceStamped>();

p->header.frame_id = global_frame_id_;
p->header.stamp = laser_scan->header.stamp;

p->pose.pose.position.x =
  hyps[max_weight_hyp].pf_pose_mean.v[0];
p->pose.pose.position.y =
  hyps[max_weight_hyp].pf_pose_mean.v[1];
p->pose.pose.orientation =
  orientationAroundZAxis(
    hyps[max_weight_hyp].pf_pose_mean.v[2]);

pf_sample_set_t * set =
  pf_->sets + pf_->current_set;

for (int i = 0; i < 2; i++) {
  for (int j = 0; j < 2; j++) {
    p->pose.covariance[6 * i + j] =
      set->cov.m[i][j];
  }
}
p->pose.covariance[6 * 5 + 5] =
  set->cov.m[2][2];

pose_pub_->publish(std::move(p));
~~~

输出规则：

~~~
位置和 yaw ─► 最高权重粒子簇的均值
协方差     ─► 当前整个粒子集合的总体协方差
~~~

ROS 使用 6×6 协方差，AMCL 主要填充二维 x/y 和 yaw：

~~~
covariance[0]、[1]、[6]、[7]  ← x/y
covariance[35]                 ← yaw
~~~

### 15.3 从 `map -> base` 反推出 `map -> odom`

AMCL 粒子簇提供地图中的机器人 pose：

~~~
map -> base
~~~

机器人运动系统通过 TF 提供：

~~~
odom -> base
~~~

两者描述同一个 `base`，因此可以反推出：

~~~
map -> odom
~~~

来源：`[src/amcl_node.cpp](../reference/nav2_amcl_jazzy/src/amcl_node.cpp)`，函数 `calculateMaptoOdomTransform`。

~~~cpp
tf2::Quaternion q;
q.setRPY(0, 0,
  hyps[max_weight_hyp].pf_pose_mean.v[2]);

tf2::Transform tmp_tf(q, tf2::Vector3(
  hyps[max_weight_hyp].pf_pose_mean.v[0],
  hyps[max_weight_hyp].pf_pose_mean.v[1],
  0.0));

geometry_msgs::msg::PoseStamped tmp_tf_stamped;
tmp_tf_stamped.header.frame_id = base_frame_id_;
tmp_tf_stamped.header.stamp =
  laser_scan->header.stamp;
tf2::toMsg(tmp_tf.inverse(),
  tmp_tf_stamped.pose);

tf_buffer_->transform(
  tmp_tf_stamped, odom_to_map,
  odom_frame_id_);

tf2::impl::Converter<true, false>::convert(
  odom_to_map.pose, latest_tf_);
latest_tf_valid_ = true;
~~~

这里做的是坐标变换代数拼接，不是再跑一遍定位算法。最新变换先缓存到 `latest_tf_`。

### 15.4 只有知道初始位姿后才发布 map -> odom

来源：同一文件，函数 `sendMapToOdomTransform`。

~~~cpp
if (!initial_pose_is_known_) {
  return;
}

geometry_msgs::msg::TransformStamped tmp_tf_stamped;
tmp_tf_stamped.header.frame_id =
  global_frame_id_;
tmp_tf_stamped.header.stamp =
  tf2_ros::toMsg(transform_expiration);
tmp_tf_stamped.child_frame_id =
  odom_frame_id_;

tf2::impl::Converter<false, true>::convert(
  latest_tf_.inverse(),
  tmp_tf_stamped.transform);

tf_broadcaster_->sendTransform(
  tmp_tf_stamped);
~~~

通常最终是：

~~~
header.frame_id  = map
child_frame_id   = odom
~~~

这不是静态 `map -> odom`，而是 AMCL 依据粒子估计动态更新的定位 TF。`transform_tolerance` 让变换在短时间窗口内仍可被下游查询。

### 15.5 一轮回调中的最终输出顺序

来源：`laserReceived` 后半段。

~~~cpp
if (getMaxWeightHyp(
    hyps, max_weight_hyps, max_weight_hyp)) {
  publishAmclPose(
    laser_scan, hyps, max_weight_hyp);

  calculateMaptoOdomTransform(
    laser_scan, hyps, max_weight_hyp);

  if (tf_broadcast_ == true) {
    auto stamp =
      tf2_ros::fromMsg(laser_scan->header.stamp);
    tf2::TimePoint expiration =
      stamp + transform_tolerance_;

    sendMapToOdomTransform(expiration);
    sent_first_transform_ = true;
  }
}
~~~

同一个最高权重假设同时产生：

~~~
最高权重粒子簇
      ├─► /amcl_pose
      └─► map -> odom TF
~~~

## 16. 把一帧激光压缩成伪代码

~~~text
on LaserScan(scan):
    如果节点不是 active：返回
    如果还没有地图：返回

    如果 scan 的 frame 第一次出现：
        创建激光模型
        查询 laser -> base 的 TF

    pose_odom = 查询 scan 时间的 base -> odom

    如果还没有 odom 基准：
        保存 pose_odom
        标记所有激光模型需要更新
    否则：
        delta = pose_odom - 上次滤波 pose
        如果 delta 超过阈值：标记需要更新

    如果当前激光模型需要更新：
        对每个粒子：
            根据 delta + 噪声更新 pose

        将 LaserScan 转成 range/bearing 数组

        对每个粒子：
            把 beam 端点投影到 map
            查询端点的障碍距离
            修改粒子 weight

        归一化权重

        如果达到 resample_interval：
            按权重复制粒子
            必要时注入地图随机粒子
            KD 树聚类并计算统计量

        发布粒子云

    选择最高权重粒子簇
    发布 /amcl_pose
    根据 map -> base 和 base -> odom 推出 map -> odom
    发布 map -> odom TF
~~~

对应的真实调用链：

~~~text
LaserScan
  ↓
tf2_ros::MessageFilter
  ↓
AmclNode::laserReceived()
  ↓
AmclNode::addNewScanner()
  ↓
AmclNode::getOdomPose()
  ↓
AmclNode::shouldUpdateFilter()
  ↓
MotionModel::odometryUpdate()
  ↓
AmclNode::updateFilter()
  ↓
Laser::sensorUpdate()
  ↓
LikelihoodFieldModel::sensorFunction()
  ↓
pf_update_sensor()
  ↓
pf_update_resample()
  ↓
pf_cluster_stats()
  ↓
AmclNode::getMaxWeightHyp()
  ↓
AmclNode::publishAmclPose()
  ↓
AmclNode::calculateMaptoOdomTransform()
  ↓
AmclNode::sendMapToOdomTransform()
~~~

## 17. 官方实现与 mini_nav 的对应关系

| 官方 AMCL | 关键文件 | mini_nav 对应位置 | 当前状态 |
|---|---|---|---|
| 位姿和粒子数据 | `pf.hpp`、`pf_vector.hpp` | `mini_nav_core/localization/localization_types.hpp` | M1 已完成基础版本 |
| 粒子初始化、权重、重采样、统计 | `pf.c`、`pf_pdf.c` | `mini_nav_core/localization/particle_filter.*` | M1 已完成 |
| 地图表示 | `map.hpp`、`map.c` | `mini_nav_core/costmap_2d.*` | 已有 Costmap2D，需适配语义 |
| 差分运动模型 | `differential_motion_model.*` | `mini_nav_core/localization/differential_motion_model.*` | 下一阶段 |
| 激光模型 | `sensors/laser/*` | `mini_nav_core/localization/laser_model.*` | 后续 |
| 定位总引擎 | `AmclNode` 中的调度逻辑 | `localization/localization_engine.*` | 后续 |
| ROS 2 适配 | `amcl_node.hpp/.cpp` | `mini_nav_nodes/amcl_node.*` | 最后接入 |
| 启动和参数 | `package.xml`、launch、配置 | `mini_nav_bringup` | 自研入口时接入 |

边界应保持：

~~~
mini_nav_core
    ROS 无关的数学和地图算法

mini_nav_nodes
    ROS 消息、QoS、TF、生命周期和发布

mini_nav_bringup
    启动仿真、地图、传感器和自研节点
~~~

官方 `AmclNode` 适合用来学习 ROS 适配层该处理哪些问题，但不应直接成为 `mini_nav_core` 的依赖。

## 18. 推荐阅读顺序

### 第 1 次：只读接口

阅读：

- `[pf.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/pf/pf.hpp)`
- `[pf_vector.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/pf/pf_vector.hpp)`
- `[laser.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/sensors/laser/laser.hpp)`
- `[motion_model.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/motion_model/motion_model.hpp)`

目标：说清楚粒子、粒子集合和模块接缝。

### 第 2 次：读粒子滤波

阅读 `pf.c` 中的：

~~~text
pf_alloc
pf_init
pf_update_sensor
pf_update_resample
pf_cluster_stats
~~~

目标：画出 `set_a -> set_b -> current_set` 的变化。

### 第 3 次：读运动模型

阅读 `[differential_motion_model.cpp](../reference/nav2_amcl_jazzy/src/motion_model/differential_motion_model.cpp)` 和 `[angleutils.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/angleutils.hpp)`。

目标：用一个 odom 增量解释 `rot1 -> trans -> rot2` 和噪声。

### 第 4 次：读地图和激光模型

阅读：

- `[map.hpp](../reference/nav2_amcl_jazzy/include/nav2_amcl/map/map.hpp)`
- `[map_cspace.cpp](../reference/nav2_amcl_jazzy/src/map/map_cspace.cpp)`
- `[likelihood_field_model.cpp](../reference/nav2_amcl_jazzy/src/sensors/laser/likelihood_field_model.cpp)`

目标：手算一条 beam 的端点如何落到地图栅格，以及 `occ_dist` 如何影响权重。

### 第 5 次：最后读 `amcl_node.cpp`

重点函数：

~~~text
on_configure
on_activate
handleInitialPose
handleMapMessage
initMessageFilters
laserReceived
shouldUpdateFilter
updateFilter
getMaxWeightHyp
publishAmclPose
calculateMaptoOdomTransform
sendMapToOdomTransform
~~~

目标：看到 ROS 代码时，能把每个调用映射回已经理解的算法模块。

## 19. 与自研实现的推进顺序

不需要一开始复刻官方 ROS 外壳。按算法接缝逐步实现：

~~~text
M1  粒子初始化、权重归一化、重采样和统计        已完成
M2  差分运动模型，给粒子应用带噪声 odom 增量
M3  Costmap2D 上的二维激光端点投影
M4  likelihood-field 激光权重模型
M5  LocalizationEngine 串起 predict/update/resample
M6  用测试数据验证收敛和错误恢复
M7  写 amcl_node，接入 map/scan/TF
M8  使用相同地图和仿真轨迹与官方 AMCL 对照
~~~

如果 M2 到 M6 的核心模块能在不启动 ROS 的情况下通过单元测试，`amcl_node.cpp` 中最复杂的生命周期、TF、QoS 和消息处理就会被隔离在最后一层。

## 20. 容易误解的地方

1. `pf_init_` 不是 `pf_`。前者是 odom 更新基准是否建立，后者是粒子滤波器对象。
2. `/amcl_pose` 和 `map -> odom` 不是两个独立定位结果，它们来自同一个最高权重粒子簇。
3. 默认 likelihood-field 模型不是实时射线追踪；它查询激光端点的预计算障碍距离。
4. 重采样不是每帧必然发生，`resample_interval` 控制其频率。
5. 随机恢复粒子来自地图自由空间，不是简单地把所有粒子统一扩大方差。
6. 地图消息转换、TF 查询和 QoS 属于 ROS 适配；粒子权重公式和运动噪声属于算法核心。

## 21. 许可和改编说明

本文包含来自官方 `nav2_amcl` 的少量学习片段。参考副本保留了源码版权头、包元数据和许可文件。

如果未来从这些片段改编代码：

1. 不要删除原始版权和许可声明。
2. 明确标注上游代码和本项目新代码的边界。
3. 将改编代码放在 `reference` 目录之外。
4. 遵守对应文件和包的 LGPL 许可要求。

本文的目的，是把成熟实现拆成可以学习和测试的小模块，而不是复制一个无法解释的巨大节点。
