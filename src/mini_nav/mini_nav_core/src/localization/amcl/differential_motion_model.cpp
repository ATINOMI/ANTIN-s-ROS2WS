/**
 * @file differential_motion_model.cpp
 * @brief 实现 DifferentialMotionModel，用于粒子滤波中的差分驱动运动预测。
 *
 * DifferentialMotionModel 的作用不是直接计算机器人的绝对位置，
 * 而是根据前后两次里程计位姿：
 *
 *   previous_odom_pose
 *   current_odom_pose
 *
 * 推算机器人在这一时间段内发生了怎样的“相对运动”。
 *
 * 对于平面差分驱动机器人，一次运动可以等效分解成：
 *
 *   1. 第一次旋转 delta_rot1
 *      从原来的朝向转到本次位移方向。
 *
 *   2. 平移 delta_trans
 *      沿本次位移方向移动。
 *
 *   3. 第二次旋转 delta_rot2
 *      从位移方向继续调整到最终朝向。
 *
 * 即：
 *
 *      原始朝向
 *         │
 *         │ delta_rot1
 *         ▼
 *      位移方向
 *         │
 *         │ delta_trans
 *         ▼
 *      新的位置
 *         │
 *         │ delta_rot2
 *         ▼
 *      最终朝向
 *
 * 里程计存在编码器误差、轮胎打滑、机械误差等问题，因此不能直接认为
 * 上述运动量是完全准确的。
 *
 * 本模型根据 alpha1 ~ alpha4 计算每个运动分量对应的噪声方差，
 * 再为每个粒子分别采样高斯噪声。
 *
 * 因此：
 *
 *   同一个里程计运动
 *
 * 会让不同粒子执行略有不同的运动，从而使粒子云能够表达机器人位姿的
 * 不确定性。
 *
 * @author Antinomy
 * @date 2026-08-31
 */

/* Includes ----------------------------------------------------------------*/

#include "mini_nav_core/localization/amcl/differential_motion_model.hpp"
#include "mini_nav_core/localization/amcl/localization_constants.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

/* Namespace ---------------------------------------------------------------*/

namespace mini_nav_core::localization
{

namespace
{

/* Functions ---------------------------------------------------------------*/

/**
 * @brief 检查运动模型噪声系数是否合法。
 *
 * DifferentialMotionModel 使用 alpha 参数控制运动噪声大小。
 *
 * 这些参数最终会参与类似下面的方差计算：
 *
 *   variance = alpha * motion²
 *
 * 因此 alpha 必须满足：
 *
 *   1. 是有限数；
 *   2. 大于等于 0。
 *
 * 不能接受： NaN，+inf，-inf，负数
 *
 * 如果允许负数进入方差计算，就可能产生负方差。
 *
 * 而 SampleGaussian() 后续需要计算 sqrt(variance) ，对负数开平方会产生非法结果。
 *
 * @param value 要检查的噪声系数。
 * @param name 参数名称，仅用于生成异常信息，例如 "alpha1"。
 *
 * @throws std::invalid_argument
 *         当 value 不是有限数或者 value < 0 时抛出。
 */
void ValidateMotionNoiseCoefficient(double value, const char * name)
{
  // std::isfinite(value) 用于判断 value 是否为正常有限数。
  //
  // 以下情况都会返回 false：
  //
  //   NaN
  //   +infinity
  //   -infinity
  //
  // 同时噪声系数不能为负数，因为噪声方差本身不能为负。
  if (!std::isfinite(value) || value < 0.0) {
    throw std::invalid_argument(
      std::string(name) + " must be finite and non-negative");
  }
}

/**
 * @brief 从均值为 0 的高斯分布中随机采样一个运动噪声。
 *
 * 输入的 variance 表示高斯分布的方差：
 *
 *   variance = σ²
 *
 * 而 std::normal_distribution 的构造函数需要 normal_distribution(mean, stddev)
 *
 * 第二个参数要求的是标准差 σ，而不是方差 σ²。
 *
 * 因此这里需要：stddev = sqrt(variance)
 *
 * 最终得到的随机噪声满足：noise ~ N(0, variance)
 *
 * 均值使用 0，表示当前模型假设随机运动误差没有固定偏向：
 *
 *   有时会偏大，有时会偏小
 *   大量采样后的平均误差趋近于 0。
 *
 * @param generator 随机数生成器。
 * @param variance 高斯分布的方差 σ²。
 *
 * @return 从指定高斯分布中随机采样得到的噪声。
 */
double SampleGaussian(std::mt19937_64 & generator, double variance)
{
  // 如果方差为 0，则表示这一运动分量没有随机噪声。
  //
  // 理论上经过前面的参数检查以后 variance 不应该小于 0，
  // 这里仍然使用 <= 0 做一层防御性处理。
  if (variance <= 0.0) {
    return 0.0;
  }

  // std::normal_distribution 要求传入标准差 σ。
  //
  // 已知 variance = σ²
  //
  // 所以 σ = sqrt(variance)
  std::normal_distribution<double> distribution(
    0.0,
    std::sqrt(variance));

  // 使用 generator 从上述高斯分布中采样一次。
  //
  // 每次调用通常都会获得不同的随机值，
  // 因此不同粒子最终会执行略有不同的运动。
  return distribution(generator);
}

}  // namespace

/**
 * @brief 构造差分驱动运动模型。
 *
 * alpha1 ~ alpha4 是经典概率里程计运动模型中的噪声系数。
 *
 * 当前实现中可以理解为：
 *
 * alpha 越大：
 *   → 表示越不信任里程计 → 对应运动产生的随机噪声越大 → 粒子云运动以后通常会扩散得更明显。
 *
 * alpha 越小：
 *   → 表示越信任里程计   → 对应运动产生的随机噪声越小 → 粒子的预测结果更加集中。
 *
 * @param alpha1 旋转运动对旋转误差的影响。
 *
 * @param alpha2 平移运动对旋转误差的影响。
 *
 * @param alpha3 平移运动对平移误差的影响。
 *
 * @param alpha4 旋转运动对平移误差的影响。
 *
 * @param alpha5 当前仅为了接口兼容而保留，
 * 本 DifferentialMotionModel 的运动计算中并没有实际使用它。
 *
 * @param seed 用于初始化随机数生成器 generator_。
 *
 * 固定 seed 可以让程序每次产生相同的伪随机数序列，
 * 对单元测试和定位算法调试很有用。
 */
DifferentialMotionModel::DifferentialMotionModel(
  double alpha1,
  double alpha2,
  double alpha3,
  double alpha4,
  double alpha5,
  std::uint64_t seed)
: alpha1_(alpha1),
  alpha2_(alpha2),
  alpha3_(alpha3),
  alpha4_(alpha4),
  alpha5_(alpha5),
  generator_(seed)
{
  // 在对象构造阶段立即检查所有噪声参数。
  //
  // 这样如果参数配置错误，可以尽早失败，
  // 而不是等运动模型运行时才出现 NaN 等难以排查的问题。
  ValidateMotionNoiseCoefficient(alpha1_, "alpha1");
  ValidateMotionNoiseCoefficient(alpha2_, "alpha2");
  ValidateMotionNoiseCoefficient(alpha3_, "alpha3");
  ValidateMotionNoiseCoefficient(alpha4_, "alpha4");

  // alpha5 当前不参与 DifferentialMotionModel 的噪声计算，
  // 仅为了保持统一 API 而保留。
  //
  // 即使当前没有使用，也仍然检查其合法性，
  // 防止接口中存在无效配置。
  ValidateMotionNoiseCoefficient(alpha5_, "alpha5");
}

/**
 * @brief 根据前后两次里程计位姿，对所有粒子执行一次运动预测。
 *
 * 该函数属于粒子滤波中的“预测阶段”。
 *
 * 输入 previous_odom_pose:  上一时刻机器人里程计估计的位姿。
 *      current_odom_pose:  当前时刻机器人里程计估计的位姿。
 *
 * @param particles
 *        粒子集合。
 *        使用非 const 引用，因为本函数会直接修改每个粒子的 pose。
 *
 * @param previous_odom_pose
 *        上一时刻的里程计位姿。
 *
 * @param current_odom_pose
 *        当前时刻的里程计位姿。
 *
 * 函数首先利用两个 odom pose 计算：
 *
 *   delta_rot1
 *   delta_trans
 *   delta_rot2
 *
 * 然后给这三个运动分量加入随机噪声，
 * 最后将带噪声的相对运动应用到每一个粒子自己的位姿上。
 *
 */
void DifferentialMotionModel::UpdateParticles(
  std::vector<Particle> & particles,
  const Pose2D & previous_odom_pose,
  const Pose2D & current_odom_pose)
{
  /* Step 1：计算两次里程计位置之间的平面位移 ---------------------------*/

  // 机器人在世界坐标系 X Y 方向上的位移：
  //
  //   delta_x/delta_y = 当前 x/y - 上一时刻 x/y
  //
  //   delta_x/delta_y 可以是正数，也可以是负数。
  const double delta_x = current_odom_pose.x - previous_odom_pose.x;
  const double delta_y = current_odom_pose.y - previous_odom_pose.y;

  /* Step 2：计算这次运动的总平移距离 ----------------------------------*/

  // std::hypot(a, b) 用于计算 sqrt(a² + b²)
  //                   ____________________
  //   delta_trans =  √ delta_x² + delta_y² ,此变量用于代表前后两个里程计位置之间的直线距离。
  //
  const double delta_trans = std::hypot(delta_x, delta_y);

  /* Step 3：计算第一次旋转 delta_rot1 ----------------------------------*/

  // 如果机器人确实发生了明显平移，那么：
  //
  //   atan2(delta_y, delta_x)
  //
  // 可以得到从 previous position 指向 current position 的方向角。
  //
  // 也就是本次运动的“位移方向”。
  //
  //
  // 举例：
  //
  //              current
  //                 ●
  //                /
  //               /   ← 位移方向
  //              /
  //     previous ●
  //
  //
  // 假设：
  //
  //   上一时刻车头朝向 = 30°
  //   本次位移方向     = 50°
  //
  // 那么：
  //
  //   delta_rot1 = 50° - 30° = 20°
  //
  // 因此 delta_rot1 表示：
  //
  //   “从机器人原来的朝向，转到本次位移方向，需要转多少角度。”
  //
  //
  // 但是如果平移距离非常小：
  //
  //   delta_trans < kMinimumTranslationForRotationM
  //
  // 那么 delta_x、delta_y 可能都只是数值噪声。
  //
  // 此时 atan2(delta_y, delta_x) 得到的所谓“位移方向”
  // 没有可靠的物理意义。
  //
  // 因此直接令：
  //
  //   delta_rot1 = 0
  //
  // 注意：
  //
  //   这并不表示机器人没有旋转。
  //
  // 如果机器人发生的是原地旋转，
  // 真实姿态变化仍然会在后面的 delta_rot2 中体现。
  const double delta_rot1 = delta_trans < kMinimumTranslationForRotationM
                            ? 0.0
                            : AngularDistance(std::atan2(delta_y, delta_x), previous_odom_pose.yaw);

  /* Step 4：计算第二次旋转 delta_rot2 ----------------------------------*/


  //   AngularDistance(current_odom_pose.yaw,previous_odom_pose.yaw)
  //
  //   该函数表示从上一时刻到当前时刻的“总姿态变化”：delta_yaw = current_yaw - previous_yaw
  //
  //   AngularDistance() 同时负责正确处理角度周期，通过归一化处理，
  //   避免例如：179° -> -179° 被错误理解成旋转了 -358°。
  //
  //   通过total_delta_yaw = delta_rot1 + delta_rot2可计算出delta_rot2
  //   在这一运动模型中，delta_rot2 负责在移动完成以后，将机器人朝向补偿到最终的 current yaw。
  const double delta_rot2 = AngularDistance(
      AngularDistance(current_odom_pose.yaw,previous_odom_pose.yaw),
      delta_rot1);

  /* Step 5：计算噪声模型使用的有效旋转量 -------------------------------*/

  // 这里比较 delta_rot1 与 0、π 的角距离，取较小值作为 rot1_noise。
  //
  // 单纯使用abs(delta_rot1)会让机器人倒车时产生巨大的旋转量，导致生成不必要的旋转噪声。
  //
  // 例如机器人车头朝东→ 但直接向西倒车←
  // 从位置变化计算出来的位移方向与车头方向大约相差 π，导致 delta_rot1 接近 π rad
  //
  // 如果直接认为机器人真的发生了 π rad 的巨大旋转，就会人为产生非常大的旋转噪声。
  //
  //  这里使用如下方法计算 rot1_noise：min(|distance(delta_rot1, 0)|, |distance(delta_rot1, π)|)
  //  从而避免把倒车误认为一次巨大的旋转。

  const double rot1_noise = std::min(std::abs(AngularDistance(delta_rot1,0.0)),
                                     std::abs(AngularDistance(delta_rot1,kPi))
                                    );

  // 第二次旋转同样需要考虑倒车运动的等效表示问题，
  // 所以使用相同方法得到 rot2_noise。

  const double rot2_noise = std::min(std::abs(AngularDistance(delta_rot2,0.0)),
                                     std::abs(AngularDistance(delta_rot2,kPi))
                                    );

  /* Step 6：根据运动量计算运动噪声方差 --------------------------------*/

  // 第一次旋转的噪声方差 rot1_variance = alpha1 * rot1² + alpha2 * trans²
  //
  //
  // alpha1 表示： “旋转运动本身会产生多少旋转误差。”
  //
  // 即：转得越多 -> 旋转误差通常越大
  //
  // alpha2 表示： “平移运动会附带产生多少旋转误差。”
  //
  // 例如差分驱动机器人长距离直线运动时，
  // 左右轮速度存在轻微差异，就可能逐渐产生方向偏差。

  const double rot1_variance = alpha1_ * rot1_noise  * rot1_noise +
                               alpha2_ * delta_trans * delta_trans;

  // 平移运动的噪声方差 trans_variance = alpha3 * trans² + alpha4 * rot1² + alpha4 * rot2²
  //
  //
  // alpha3 表示：平移运动本身会产生多少平移误差。
  //
  // 例如：编码器累计误差
  //      轮胎打滑
  //      轮径误差
  //
  // 都会使移动距离越长时误差通常越明显。
  //
  //
  // alpha4 表示：旋转运动会附带产生多少平移误差。
  //
  // 例如差分驱动机器人旋转时，由于轮胎与地面的摩擦和打滑，实际旋转中心可能发生轻微漂移。
  // 因此旋转也可能产生额外的平移误差。

  const double trans_variance = alpha3_ * delta_trans * delta_trans +
                                alpha4_ * rot1_noise  * rot1_noise  +
                                alpha4_ * rot2_noise  * rot2_noise;

  // 第二次旋转的噪声模型与第一次旋转一致：
  // rot2_variance = alpha1 * rot2² + alpha2 * trans²
  //
  // 即：旋转越大 → 旋转误差越大
  //    平移越远 → 也可能附带产生更多方向误差

  const double rot2_variance = alpha1_ * rot2_noise * rot2_noise +
                               alpha2_ * delta_trans * delta_trans;

  /* Step 7：对每个粒子分别采样运动噪声 -------------------------------*/

  // particles 中的每一个粒子都表示 机器人真实位姿的一种可能假设。
  //
  // 如果所有粒子都使用：delta_rot1
  //                  delta_trans
  //                  delta_rot2
  //
  // 完全相同地移动，那么运动模型就无法表达里程计的不确定性。
  //
  // 所以所有粒子不能完全执行同一个确定运动,而是每个粒子都应该根
  //   据运动噪声方差，独立采样随机噪声。
  //
  // 因此这里对每一个粒子都重新采样随机噪声：
  //
  //   particle A
  //      rot1 = 10.2°
  //      trans = 0.98 m
  //
  //   particle B
  //      rot1 = 9.7°
  //      trans = 1.01 m
  //
  //   particle C
  //      rot1 = 10.5°
  //      trans = 1.03 m
  //
  // 最终粒子云自然发生一定程度的扩散。
  for (auto & particle : particles) {

    // 为第一次旋转加入高斯随机噪声。
    //
    // SampleGaussian() 返回：
    //
    //   error_rot1 ~ N(0, rot1_variance)
    //
    // AngularDistance() 同时保证结果仍然是合理的周期角度。
    //
    // 这里使用“减去噪声”：noisy_rot1 = delta_rot1 - error
    //
    // 因为零均值高斯分布关于 0 对称，
    // 使用 +error 或 -error 在概率分布意义上是等价的。
    const double noisy_rot1 = AngularDistance(delta_rot1, SampleGaussian(generator_, rot1_variance));

    // 为平移距离加入高斯随机噪声。
    //
    // 例如：
    //
    //   里程计认为移动 1.00 m
    //
    // 不同粒子可能得到：
    //
    //   0.98 m
    //   1.02 m
    //   0.96 m
    //   1.01 m
    //
    // 用来表达：
    //
    //   “机器人到底移动了多远存在不确定性。”
    const double noisy_trans = delta_trans - SampleGaussian(generator_, trans_variance);

    // 为第二次旋转同样加入独立的高斯随机噪声。
    const double noisy_rot2 = AngularDistance(delta_rot2, SampleGaussian(generator_, rot2_variance));

    /* Step 8：使用带噪声的运动更新粒子位姿 -----------------------------*/

    // 当前粒子原本的朝向：particle.pose.yaw
    //
    // 经过第一次旋转以后，本次平移方向变为： particle.pose.yaw + noisy_rot1
    //
    // 将 noisy_trans 投影到世界坐标系 X 轴： delta_x_particle = noisy_trans * cos(yaw + noisy_rot1)
    //
    // 所以新的 X 坐标：new_x = old_x + delta_x_particle
    particle.pose.x += noisy_trans * std::cos(particle.pose.yaw + noisy_rot1);

    // 同理，将平移距离投影到世界坐标系 Y 轴： delta_y_particle = noisy_trans * sin(yaw + noisy_rot1)
    //
    // 所以新的 Y 坐标：new_y = old_y + delta_y_particle
    particle.pose.y += noisy_trans * std::sin(particle.pose.yaw + noisy_rot1);

    // 最后更新粒子的朝向。
    //   new_yaw = old_yaw + noisy_rot1 + noisy_rot2
    //
    // NormalizeAngle() 用于将角度规范到项目约定的范围，
    // 避免多次累计以后出现类似：5π,-8π,20π,这种数学上等价、但不方便后续计算的角度。
    particle.pose.yaw = NormalizeAngle(particle.pose.yaw + noisy_rot1 + noisy_rot2);
  }
}

}  // namespace mini_nav_core::localization
