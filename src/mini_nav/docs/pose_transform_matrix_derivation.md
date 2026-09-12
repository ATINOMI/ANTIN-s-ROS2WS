# Pose2D 位姿复合的矩阵推导

## 1. 函数作用

`ComposePose` 不是专门计算 `odom -> base` 的函数，而是一个通用的二维位姿复合函数。

它把一个基座位姿和一个在基座坐标系中表达的相对位姿组合起来，得到相对位姿在父坐标系中的结果：

```text
T_parent_child = T_parent_base × T_base_child
```

其中：

- `base` 表示基座坐标系在父坐标系中的位姿。
- `relative` 表示目标坐标系在基座坐标系中的相对位姿。
- 返回值表示目标坐标系在父坐标系中的位姿。

## 2. Pose2D 的齐次变换矩阵

设二维位姿为：

```text
Pose2D(x, y, yaw)
```

它对应的 3×3 齐次变换矩阵为：

$$
T(x, y, \theta) =
\begin{bmatrix}
\cos\theta & -\sin\theta & x \\
\sin\theta &  \cos\theta & y \\
0           &  0            & 1
\end{bmatrix}
$$

矩阵左上角的 2×2 部分表示旋转，最后一列表示平移。

对于父坐标系 `P`、基座坐标系 `B` 和目标坐标系 `C`：

```text
base     = T_P_B
relative = T_B_C
```

## 3. 位姿复合

两个变换按坐标系链相乘：

$$
T_{P C} = T_{P B} T_{B C}
$$

令：

$$
T_{P B} =
\begin{bmatrix}
c_b & -s_b & x_b \\
s_b &  c_b & y_b \\
0   &  0   & 1
\end{bmatrix}
$$

$$
T_{B C} =
\begin{bmatrix}
c_r & -s_r & x_r \\
s_r &  c_r & y_r \\
0   &  0   & 1
\end{bmatrix}
$$

其中：

```text
c_b = cos(base.yaw)
s_b = sin(base.yaw)
c_r = cos(relative.yaw)
s_r = sin(relative.yaw)
```

矩阵相乘后，平移部分为：

$$
x = x_b + c_b x_r - s_b y_r
$$

$$
y = y_b + s_b x_r + c_b y_r
$$

旋转部分满足：

$$
R(\theta_b)R(\theta_r) = R(\theta_b + \theta_r)
$$

因此：

$$
yaw = NormalizeAngle(yaw_b + yaw_r)
$$

## 4. 与 C++ 函数逐项对应

```cpp
Pose2D ComposePose(const Pose2D & base, const Pose2D & relative)
{
  // base 是基座坐标系在父坐标系中的位姿。
  // relative 是目标坐标系在基座坐标系中的相对位姿。
  // 返回目标坐标系在父坐标系中的位姿。

  const double cosine = std::cos(base.yaw);
  const double sine = std::sin(base.yaw);

  return Pose2D{
    // 将 relative 的平移从基座坐标系旋转到父坐标系，
    // 再加上基座原点在父坐标系中的位置。
    base.x + cosine * relative.x - sine * relative.y,
    base.y + sine * relative.x + cosine * relative.y,

    // 复合两个坐标系的朝向，并把结果归一化到约定范围。
    NormalizeAngle(base.yaw + relative.yaw)};
}
```

注意，`relative.x` 和 `relative.y` 必须是在 `base` 坐标系中表达的平移。如果它们已经是在父坐标系中表达的平移，就不应该再次乘以 `base` 的旋转矩阵。

## 5. 一个数值例子

设：

```text
base     = (10, 5, π/2)
relative = (1, 0, 0)
```

基座朝向为 `π/2`，所以基座坐标系中的前方对应父坐标系的 `+y` 方向：

$$
x = 10 + \cos(\pi/2) \times 1 - \sin(\pi/2) \times 0 = 10
$$

$$
y = 5 + \sin(\pi/2) \times 1 + \cos(\pi/2) \times 0 = 6
$$

结果为：

```text
(10, 6, π/2)
```

## 6. 在 mini_nav AMCL 中的实际含义

当前 `likelihood_field_model.cpp` 和 `beam_model.cpp` 中的调用是：

```cpp
const Pose2D laser_pose = ComposePose(particle.pose, laser_pose_in_base);
```

对应的坐标变换链为：

```text
particle.pose       = T_map_base
laser_pose_in_base  = T_base_laser
结果                = T_map_laser
```

也就是说，粒子表示机器人基座在地图中的假设位姿，`laser_pose_in_base` 表示激光雷达相对于基座的安装位姿，函数将两者组合成激光雷达在地图中的位姿。随后激光端点才能被投影到地图坐标系中。

## 7. 它与 odom -> base 的关系

`odom -> base` 是一个具体的 TF 关系，而 `ComposePose` 是通用的矩阵复合操作。

如果已有：

```text
T_odom_A
T_A_base
```

可以通过：

```text
T_odom_base = T_odom_A × T_A_base
```

调用 `ComposePose` 得到 `odom -> base`。

但在当前 AMCL 中，`T_odom_base` 通常由 Gazebo 或里程计 TF 提供。AMCL 查询它来计算运动增量，然后根据：

```text
T_map_base
T_odom_base
```

计算定位 TF：

$$
T_{map\to odom} = T_{map\to base} \left(T_{odom\to base}\right)^{-1}
$$

## 8. 变换矩阵的逆

对于：

$$
T =
\begin{bmatrix}
c & -s & x \\
s &  c & y \\
0 &  0 & 1
\end{bmatrix}
$$

其逆矩阵为：

$$
T^{-1} =
\begin{bmatrix}
c & s & -cx - sy \\
-s & c & sx - cy \\
0 & 0 & 1
\end{bmatrix}
$$

逆矩阵的含义是把“父坐标系中的表达”转换回“子坐标系中的表达”。旋转矩阵取转置，平移部分需要先经过逆旋转，再取负号。

## 9. 最终记忆方式

```text
先把 relative 的位置按 base 的朝向旋转，
再加上 base 的位置；
两个 yaw 相加并归一化。
```

矩阵形式就是：

```text
T_parent_child = T_parent_base × T_base_child
```
