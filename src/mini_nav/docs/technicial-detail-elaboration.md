那我们把剩下的几个源码细节串起来。

## 1. MessageFilter：保证激光和 TF 是同一时刻的

AMCL 收到一帧激光时，不能直接拿当前 TF 使用。

因为激光消息有自己的时间戳：

```text
LaserScan.stamp = t
```

AMCL 需要找到：

```text
t 时刻的 odom → laser
```

`MessageFilter` 会等待 TF 准备好之后，才调用激光回调。

否则可能出现：

```text
激光是旧时刻的数据
TF 却是当前时刻的数据
```

这样计算出来的粒子权重就会错误。

## 2. 地图距离场：激光模型真正查询的东西

AMCL 不会为每个粒子、每条激光束都重新搜索最近墙壁。

它会提前为地图计算：

```text
每个地图栅格到最近障碍物的距离
```

例如：

```text
墙壁格：0
离墙 1 格：0.05 m
离墙 2 格：0.10 m
```

激光端点落到某个地图格后，直接查询这个格子的 `occ_dist`。

所以：

```text
地图预计算距离场
        ↓
激光端点查表
        ↓
计算粒子权重
```

这就是 likelihood field 高效的原因。

## 3. KLD 重采样：决定需要多少粒子

KD 树统计当前粒子占据了多少个空间格子：

```text
k = 非空位姿格数量
```

然后 KLD 公式根据 `k` 计算需要多少粒子：

```text
粒子集中 → k 小 → 需要的粒子较少
粒子分散 → k 大 → 需要的粒子较多
```

但结果会被限制在：

```text
min_particles ≤ N ≤ max_particles
```

因此 KD 树不仅用于聚类，也帮助 AMCL 动态决定粒子数量。

## 4. 协方差：描述定位还有多不确定

粒子平均位置表示：

```text
机器人估计在哪里
```

协方差表示：

```text
这个估计有多不确定
```

例如：

```text
粒子很集中 → 协方差小 → 定位比较确定
粒子很分散 → 协方差大 → 定位不确定
```

角度不能直接普通平均：

```text
179° 和 -179°
```

普通平均会得到 `0°`，这是错误的。

AMCL 使用：

```text
atan2(平均 sin(theta), 平均 cos(theta))
```

来计算平均朝向。

## 5. 参数分别影响哪一部分

可以这样对应：

```text
alpha1 ~ alpha5
    → 里程计运动噪声

update_min_d / update_min_a
    → 移动多少以后才触发一次滤波更新

max_beams
    → 每帧使用多少条激光束

sigma_hit / z_hit / z_rand
    → 激光模型的形状和各项权重

laser_likelihood_max_dist
    → 激光端点查询墙壁距离的最大范围

min_particles / max_particles
    → 粒子数量上下限

resample_interval
    → 多少次传感器更新进行一次重采样

recovery_alpha_fast / slow
    → 是否注入随机粒子进行定位恢复
```

## 6. 最终位姿是怎么出来的

AMCL 不一定取所有粒子的简单平均。

它会：

```text
1. 用 KD 树对粒子聚类
2. 计算每个簇的总权重
3. 选择最高权重簇
4. 使用这个簇的平均位姿作为定位结果
```

所以当地图存在多个相似位置时：

```text
簇 A：权重 0.7
簇 B：权重 0.3
```

AMCL 会优先发布簇 A 的平均位姿。

## 7. 迁移到我们自己的 mini_nav_core

理解到这里，就可以把 ROS 和算法分开：

```text
mini_nav_core
    Particle
    MotionModel
    LaserModel
    DistanceField
    ParticleFilter
    KDTree / Cluster

mini_nav_nodes
    订阅 LaserScan
    获取 TF 和里程计
    转换 ROS 消息
    调用 core
    发布 Pose、TF、ParticleCloud
```

核心算法的主接口可以是：

```text
update_motion(odom_delta)
update_sensor(scan, map, laser_pose)
resample()
get_pose_estimate()
get_covariance()
```

也就是说，剩余内容已经从“理解 AMCL 算法”转向了：

> 如何把官方 AMCL 的各个模块拆成我们自己可以实现、测试和替换的 ROS 无关组件。