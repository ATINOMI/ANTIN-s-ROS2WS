/**
 * @file kd_tree.cpp
 * @brief 粒子位姿离散分箱与空间索引的实现。
 *
 * 本文件用于将粒子滤波中的连续二维位姿状态：
 *
 *     (x, y, yaw)
 *
 * 按照给定的线性分辨率和角度分辨率离散为三维网格：
 *
 *     (x_bin, y_bin, yaw_bin)
 *
 * 每个离散网格使用 PoseBinKey 表示，并通过 std::unordered_map 建立：
 *
 *     PoseBinKey -> bin_id
 *
 * 的映射，从而可以快速判断某个空间分箱是否已经存在。
 *
 * 同时维护两个方向的粒子索引：
 *
 *     bin_id_by_particle_index_[particle_id] -> bin_id
 *
 * 用于查询某个粒子属于哪个分箱；
 *
 *     particle_indices_by_bin_id_[bin_id] -> [particle_id...]
 *
 * 用于查询某个分箱中包含哪些粒子。
 *
 * 注意：
 * 当前实现虽然类名为 PoseBinIndex，但实际上没有使用传统 Kd-tree 的
 * 二叉树节点、维度切分和递归搜索结构，而是采用
 * “空间离散化 + 哈希表”的方式实现粒子分箱索引。
 *
 * @author Antinomy
 * @date 2026-09-10
 */

/* Includes ----------------------------------------------------------------*/
#include "mini_nav_core/localization/kd_tree.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

/* Namespace ----------------------------------------------------------------*/
namespace mini_nav_core::localization
{

/* Functions ----------------------------------------------------------------*/

/**
 * @brief 构造粒子位姿分箱索引对象。
 *
 * translation_bin_size_m 用于控制 x、y 两个平移方向上的分箱大小。
 * 例如 translation_bin_size_m = 0.5 时：
 *
 *     x ∈ [0.0, 0.5)  -> x_bin = 0
 *     x ∈ [0.5, 1.0)  -> x_bin = 1
 *     x ∈ [1.0, 1.5)  -> x_bin = 2
 *
 * yaw_bin_size_rad 用于控制 yaw 方向上的角度分箱大小。
 * 例如 yaw_bin_size_rad = 15° 时，每 15° 会划分为一个新的角度分箱。
 *
 * 两个分箱尺寸必须：
 *
 * 1. 是有限值，不能是 NaN 或 ±Inf；
 * 2. 大于 0，因为后续需要使用它们作为除数进行离散化。
 *
 * @param translation_bin_size_m x、y 方向的线性分箱尺寸。
 * @param yaw_bin_size_rad yaw 方向的角度分箱尺寸，单位为弧度。
 *
 * @throws std::invalid_argument
 *         当 translation_bin_size_m 或 yaw_bin_size_rad 非有限值，
 *         或者小于等于 0 时抛出。
 */
PoseBinIndex::PoseBinIndex(
  double translation_bin_size_m, double yaw_bin_size_rad)
: translation_bin_size_m_(translation_bin_size_m),
  yaw_bin_size_rad_(yaw_bin_size_rad)
{
  // std::isfinite() 用于排除 NaN、+Inf 和 -Inf。
  //
  // 分箱尺寸必须严格大于 0：
  //
  //   1. 等于 0 会导致 MakePoseBinKey() 中发生除零；
  //   2. 小于 0 虽然数学上仍然可以做除法，
  //      但会破坏“分箱尺寸”的物理意义。
  //
  // 在构造阶段就拒绝非法参数，可以保证对象创建成功以后，
  // translation_bin_size_m_ 和 yaw_bin_size_rad_ 始终处于合法状态。
  if (!std::isfinite(translation_bin_size_m_) ||
      translation_bin_size_m_ <= 0.0 ||
      !std::isfinite(yaw_bin_size_rad_) ||
      yaw_bin_size_rad_ <= 0.0)
  {
    throw std::invalid_argument(
      "PoseBinIndex bin sizes must be finite and greater than zero");
  }
}

/**
 * @brief 根据当前粒子集合重新构建整个分箱索引。
 *
 * BuildPoseBinIndex() 会遍历所有粒子，并执行以下操作：
 *
 * 1. 将粒子的连续位姿 (x, y, yaw) 转换为离散 PoseBinKey；
 * 2. 判断该 PoseBinKey 对应的空间分箱是否已经存在；
 * 3. 如果不存在，则创建新的 bin，并为其分配 bin_id；
 * 4. 记录“粒子属于哪个 bin”；
 * 5. 记录“bin 中包含哪些粒子”。
 *
 * 最终会建立三个互相关联的数据结构：
 *
 *     bin_id_by_key_                 哈希表，存储箱子键和对应的粒子索引
 *     PoseBinKey -> bin_id
 *
 *     bin_id_by_particle_index_           存储每个粒子所属的箱子索引
 *     particle_id -> bin_id
 *
 *     particle_indices_by_bin_id_           存储每个箱子中的粒子索引
 *     bin_id -> particle_id[]
 *
 * 其中 bin_id 使用从 0 开始的连续整数编号。
 *
 * @param particles 当前需要建立索引的粒子集合。
 *
 * @note particles 通过 const 引用传入：
 *
 *       const std::vector<Particle> &
 *
 *       因此 BuildPoseBinIndex() 不会复制整个粒子数组，也不会修改调用者的粒子数据。
 */
void PoseBinIndex::BuildPoseBinIndex(const std::vector<Particle> & particles)
{
  /*
   * 粒子滤波每次更新之后，粒子的 pose 都可能发生变化。
   *
   * 因此之前建立的：
   *
   *     Particle -> Bin
   *     Bin -> Particle
   *
   * 映射已经可能失效。
   *
   * BuildPoseBinIndex() 采用“完全重建”的策略，所以首先清空上一轮索引。
   *
   * clear() 会删除容器中的元素，但对于 vector 来说通常不会立即释放
   * 已经申请的 capacity，因此下一次 BuildPoseBinIndex() 仍有机会复用已有内存。
   */
  bin_id_by_key_.clear();
  bin_id_by_particle_index_.clear();
  particle_indices_by_bin_id_.clear();

  /*
   * 每一个输入粒子最终都会在 bin_id_by_particle_index_ 中保存一个 bin_id，用来表示对应箱子的索引。
   *
   * 因此最终一定有：
   *
   *     bin_id_by_particle_index_.size() == particles.size()
   *
   * 提前 reserve() 相同数量的空间，可以减少 push_back() 过程中
   * vector 因容量不足而产生的重复扩容和数据搬移。
   *
   * 注意：
   *
   *     reserve() 只改变 capacity，
   *     不改变 size。
   */
  bin_id_by_particle_index_.reserve(particles.size());

  /*
   * for循环遍历所有粒子，同时将 index 作为当前粒子的 particle_id。
   *
   * 例如：
   *
   *     particles[0] -> particle_id = 0
   *     particles[1] -> particle_id = 1
   *     particles[2] -> particle_id = 2
   */
  for (std::size_t index = 0; index < particles.size(); ++index)
  {
    /*
     * 将连续位姿：
     *
     *     (x, y, yaw)
     *  
     *     使用 MakePoseBinKey() 函数转换为离散空间坐标：
     *
     *     PoseBinKey{x_bin, y_bin, yaw_bin}
     *
     * 两个位姿只要落在同一个离散网格中，
     * 即使它们的实际浮点数坐标不同，也会得到相同的 PoseBinKey。
     */
    const PoseBinKey key = MakePoseBinKey(particles[index].pose);

    /*
     * 尝试向 bin_id_by_key_ 哈希表中插入：
     *
     *     key -> particle_indices_by_bin_id_.size() : [键 -> 箱子编号]
     *
     * particle_indices_by_bin_id_.size() 当前正好等于已经创建的 bin 数量，
     * 因此也可以作为下一个新 bin 的编号。
     *
     * 例如当前已经存在：
     *
     *     bin 0
     *     bin 1
     *     bin 2
     *
     * 此时：
     *
     *     particle_indices_by_bin_id_.size() == 3
     *
     * 那么下一个新创建的 bin 就应该编号为 3。
     *
     * unordered_map::emplace() 返回：
     *
     *     std::pair<iterator, bool>  ： [键, 是否插入成功]
     *
     * 通过结构化绑定拆成：
     *
     *     iterator
     *     inserted
     *
     * inserted == true：
     *     key 原本不存在，本次成功创建了一个新分箱。
     *
     * inserted == false：
     *     key 已经存在，没有发生新的插入。
     *
     * 无论是否插入成功，iterator 都会指向最终对应的 map 元素，
     * 所以：
     *
     *     iterator->second
     *
     * 始终可以获得该 PoseBinKey 对应的 bin_id。
     */
    const auto [iterator, inserted] =
      bin_id_by_key_.emplace(key, particle_indices_by_bin_id_.size());

    /*
     * 如果这是一个之前从未出现过的 PoseBinKey，表示bin第一次出现，则表示插入成功
     * bin_id_by_key_ 中已经通过emplace为它创建了新的 bin_id。
     *
     * 此时还需要在 particle_indices_by_bin_id_ 中创建对应的空容器：
     *
     *     particle_indices_by_bin_id_[bin_id]
     *
     * 用来保存之后落入该分箱的 particle_id。
     *
     * emplace_back() 在 vector 末尾直接构造一个空的：
     *
     *     std::vector<std::size_t>
     */
    if (inserted)
    {
      particle_indices_by_bin_id_.emplace_back();
    }

    /*
     * iterator->second 就是当前粒子所属的 bin_id，表示该粒子所在的箱子编号。
     *
     * 建立第一个方向的映射：
     *
     *     particle_id -> bin_id
     *
     * 因为粒子按照 index = 0, 1, 2, ... 的顺序遍历并 push_back，
     * 所以最终满足：
     *
     *     bin_id_by_particle_index_[index]
     *
     * 就是 particles[index] 所属的 bin_id。
     */
    bin_id_by_particle_index_.push_back(iterator->second);

    /*
     * 建立反方向映射：
     *
     *     bin_id -> particle_id[]
     *
     * iterator->second：
     *     当前粒子所属的 bin_id。
     *
     * index：
     *     当前粒子的 particle_id。
     *
     * 因此这句话表示：
     *
     *     把当前粒子丢到对应的分箱中。
     *
     * 例如：
     *
     *     particle_indices_by_bin_id_[2] = {1, 4, 7}
     *
     * 表示编号为 1、4、7 的三个粒子都位于 bin 2。
     */
    particle_indices_by_bin_id_[iterator->second].push_back(index);
  }
}

/**
 * @brief 获取当前被至少一个粒子占用的分箱数量。
 *
 * particle_indices_by_bin_id_ 中只会为实际出现过的 PoseBinKey 创建元素。
 *
 * 因此：
 *
 *     particle_indices_by_bin_id_.size()
 *
 * 正好等于当前粒子集合占用了多少个不同的
 * (x_bin, y_bin, yaw_bin) 三维离散状态。
 *
 * 例如：
 *
 *     1000 个粒子
 *
 * 可能只有：
 *
 *     72 个不同的 PoseBinKey
 *
 * 那么本函数返回 72。
 *
 * @return 当前被粒子占用的离散分箱数量。
 */
std::size_t PoseBinIndex::GetOccupiedBinCount() const
{
  return particle_indices_by_bin_id_.size();
}

/**
 * @brief 计算 PoseBinKey 的哈希值，供 std::unordered_map 使用。
 *
 * PoseBinKey 由三个整数组成：
 *
 *     PoseBinKey{x, y, yaw}
 *
 * std::unordered_map 无法自动知道应该如何对自定义 PoseBinKey 类型进行哈希，
 * 因此需要提供 PoseBinKeyHash。
 *
 * 本函数分别计算：
 *
 *     hash(x)
 *     hash(y)
 *     hash(yaw)
 *
 * 然后使用 hash-combine 方法将三个哈希值混合成一个
 * std::size_t 类型的最终哈希值。
 *
 * 哈希值的作用只是帮助 unordered_map 快速定位内部 bucket，
 * 它不是 bin_id，也不要求不同 PoseBinKey 一定产生不同的哈希值。
 *
 * 即：
 *
 *     PoseBinKey -> hash -> unordered_map bucket
 *
 * 和：
 *
 *     PoseBinKey -> bin_id
 *
 * 是两件不同的事情。
 *
 * 如果两个不同 PoseBinKey 得到了相同 hash，这称为哈希碰撞。
 * unordered_map 会继续使用 PoseBinKey::operator== 判断两个 PoseBinKey 是否真正相等。
 *
 * @param key 要进行哈希计算的分箱 PoseBinKey。
 * @return 用于 unordered_map 内部索引的哈希值。
 */
std::size_t PoseBinIndex::PoseBinKeyHash::operator()(const PoseBinKey & key) const
{
  /*
   * 首先使用 x 维度初始化 hash。
   *
   * std::hash<int>{}(key.x_bin)
   *
   * 可以理解为：
   *
   *     int x
   *       ↓
   *     整数哈希函数
   *       ↓
   *     std::size_t
   */
  std::size_t hash = std::hash<int>{}(key.x_bin);

  /*
   * 将 y 维度的哈希值混入当前 hash。
   *
   * 如果只是简单使用：
   *
   *     hash ^= hash(y)
   *
   * 某些规律性的输入可能产生较差的哈希分布。
   *
   * 因此这里使用经典的 hash-combine 形式：
   *
   *     hash(y)
   *       +
   *     kHashCombineConstant
   *       +
   *     (hash << 6U)
   *       +
   *     (hash >> 2U)
   *
   * 其中：
   *
   *     kHashCombineConstant
   *
   * 是常用于哈希混合的常量，用于改善不同输入之间的分布。
   *
   *     hash << 6U
   *
   * 将当前 hash 左移 6 位；
   *
   *     hash >> 2U
   *
   * 将当前 hash 右移 2 位。
   *
   * 这样可以让原 hash 的不同 bit 更充分地参与下一轮混合，
   * 降低大量结构相似的 PoseBinKey 集中到相同哈希区域的概率。
   *
   * 最后的 ^= 是按位异或并赋值：
   *
   *     hash ^= value
   *
   * 等价于：
   *
   *     hash = hash ^ value;
   */
  hash ^=
    std::hash<int>{}(key.y_bin) +
    kHashCombineConstant +
    (hash << 6U) +
    (hash >> 2U);

  /*
   * 使用完全相同的方法继续把 yaw 维度混入 hash。
   *
   * 最终得到的 hash 同时受到：
   *
   *     key.x_bin
   *     key.y_bin
   *     key.yaw_bin
   *
   * 三个成员的影响。
   */
  hash ^=
    std::hash<int>{}(key.yaw_bin) +
    kHashCombineConstant +
    (hash << 6U) +
    (hash >> 2U);

  return hash;
}

/**
 * @brief 将连续二维位姿转换为离散三维分箱 PoseBinKey。
 *
 * 输入 Pose2D：
 *
 *     pose.x
 *     pose.y
 *     pose.yaw
 *
 * 输出 PoseBinKey：
 *
 *     PoseBinKey{x_bin, y_bin, yaw_bin}
 *
 * 这里的 PoseBinKey 并不是实际坐标，而是粒子所在分箱的整数编号。
 *
 * 例如：
 *
 *     translation_bin_size_m_ = 0.5
 *     pose.x = 1.27
 *
 * 那么：
 *
 *     floor(1.27 / 0.5)
 *     = floor(2.54)
 *     = 2
 *
 * 所以：
 *
 *     x_bin = 2
 *
 * 即该粒子位于 x 方向编号为 2 的分箱。
 *
 * @param pose 要进行离散化的二维位姿。
 * @return 位姿所在离散空间对应的 PoseBinKey。
 */
PoseBinIndex::PoseBinKey PoseBinIndex::MakePoseBinKey(const Pose2D & pose) const
{
  const auto to_bin_index = [](double coordinate, double bin_size, const char * name) 
  {

    if (!std::isfinite(coordinate)) 
    {
      throw std::invalid_argument(std::string("Pose coordinate is not finite: ") + name);
    }

    const double bin = std::floor(coordinate / bin_size);

    if (!std::isfinite(bin) ||
        bin < static_cast<double>(std::numeric_limits<int>::min()) ||
        bin > static_cast<double>(std::numeric_limits<int>::max())) 
    {
      throw std::invalid_argument(std::string("Pose coordinate bin is outside int range: ") + name);
    }

    return static_cast<int>(bin);
  };

  const double normalized_yaw = NormalizeAngle(pose.yaw);
  return PoseBinKey{
    /*
     * x 方向离散化：
     *
     *     x_bin = floor(x / translation_bin_size_m_)
     *
     * floor() 使用向下取整，而不是简单截断。
     *
     * 这一点对于负坐标尤其重要。
     *
     * 例如 translation_bin_size_m_ = 1：
     *
     *     x =  0.3 -> floor( 0.3) =  0
     *     x = -0.3 -> floor(-0.3) = -1
     *
     * 因此能够正确形成以整数边界划分的连续空间网格。
     */
    to_bin_index(pose.x, translation_bin_size_m_, "x"),

    /*
     * y 方向采用和 x 完全相同的离散化方法。
     */
    to_bin_index(pose.y, translation_bin_size_m_, "y"),

    /*
     * yaw 的处理比 x、y 多一步角度归一化。
     *
     * 首先：
     *
     *     NormalizeAngle(pose.yaw)
     *
     * 将任意角度归一化到约：
     *
     *     [-π, π]
     *
     * 的范围。
     *
     * 接下来加上 π：
     *
     *     yaw + π
     *
     * 将范围整体平移为：
     *
     *     [0, 2π]
     *
     * 这样角度分箱可以从接近 0 的编号开始。
     *
     * 最后：
     *
     *     floor((yaw + π) / yaw_bin_size_rad_)
     *
     * 得到 yaw 方向的离散分箱编号。
     *
     * 例如每 15° 一个分箱：
     *
     *     [-180°, -165°) -> bin 0
     *     [-165°, -150°) -> bin 1
     *     ...
     *     [0°, 15°)      -> 某个对应的 bin
     *
     * 因此最终 PoseBinKey 的三个成员分别描述粒子在：
     *
     *     x
     *     y
     *     yaw
     *
     * 三个状态维度上的离散位置。
     */
    to_bin_index(normalized_yaw + kPi, yaw_bin_size_rad_, "yaw")
  };
}

}  // namespace mini_nav_core::localization