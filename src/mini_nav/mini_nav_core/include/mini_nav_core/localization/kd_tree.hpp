/**
 * @file kd_tree.hpp
 * @brief PoseBinIndex 类的声明，用于在粒子滤波中进行高效的空间索引和查询。
 *
 * 该类使用哈希表将粒子按位姿（x、y、yaw）离散分箱，以便快速统计占用分箱。
 * 主要功能包括构建 Kd 树、获取占用的箱子数量等。
 *
 * @author Antinomy
 * @date 2026-09-10
 */
#pragma once

/* Includes ----------------------------------------------------------------*/

#include <cstddef>       // 用于 std::size_t
#include <unordered_map> // 用于存储箱子和粒子索引的哈希表
#include <vector>       

#include "mini_nav_core/localization/localization_constants.hpp"
#include "mini_nav_core/localization/types.hpp"

/* Namespace ----------------------------------------------------------------*/

namespace mini_nav_core::localization
{

  /**
   * @class PoseBinIndex
   * @brief 三维位姿分箱索引，用于粒子滤波的空间统计。
   *
   * 该类将粒子根据其位姿（x, y, yaw）进行分箱，以便快速查询和统计。
   * 主要功能包括构建 Kd 树、获取占用的箱子数量等。
   */
class PoseBinIndex
{
public:
/* Public Functions --------------------------------------------------------- */

  /**
     * @brief 构造函数，初始化平移和 yaw 分箱大小。
     *
     * @param translation_bin_size_m 线性分箱的大小（单位：米）
     * @param yaw_bin_size_rad 角度分箱的大小（单位：弧度）
     */
  explicit PoseBinIndex(
    double translation_bin_size_m = kDefaultTranslationBinSizeM,
    double yaw_bin_size_rad = kDefaultYawBinSizeRad);

  /**
   * @brief 构建位姿分箱索引，将粒子根据其位姿进行分箱。
   *
   * @param particles 引用的粒子向量，每个粒子包含位姿和权重
   */
  void BuildPoseBinIndex(const std::vector<Particle> & particles);

  // Compatibility wrapper for existing callers.
  /**
   * @brief 通过兼容名称重新构建位姿分箱索引。
   *
   * @param particles 粒子集，后续主簇查询须保持相同数量和顺序。
   * @throws std::invalid_argument 位姿非有限或量化索引超出整数范围。
   */
  void Build(const std::vector<Particle> & particles) { BuildPoseBinIndex(particles); }

  /**
   * @brief 获取被占用的格子数量。
   *
   * @return std::size_t 被占用的格子的数量
   */
  std::size_t GetOccupiedBinCount() const;

  /**
   * @brief 按相邻位姿分箱的连通性选择总权重最大的定位主簇。
   *
   * 只连通正权重分箱，避免零权重格把分离假设桥接；yaw 邻接在 ±π 处周期连接。
   *
   * @param particles 与最近一次 Build 顺序及数量一致的粒子集，权重有限非负。
   * @return 主簇中的原始粒子索引；用于位姿估计而非全部粒子的跨模式平均。
   * @throws std::invalid_argument 粒子数量与索引不匹配或权重非法。
   */
  std::vector<std::size_t> GetDominantParticleIndices(const std::vector<Particle> & particles) const;

private:
/* Private Types ------------------------------------------------------------*/


  /**
   * @brief 表示一个三维箱子的键，用于在哈希表中索引。
   *
   * 该结构体包含三个整数，分别表示箱子在 x、y 和 yaw 方向上的索引。
   */
  struct PoseBinKey
  {
    int x_bin;    // x 方向的箱子索引
    int y_bin;    // y 方向的箱子索引
    int yaw_bin;  // yaw 方向的箱子索引

    /**
     * @brief 重载相等运算符，用于比较两个 PoseBinKey 是否相等。
     *
     * @param other 另一个 PoseBinKey 对象
     * @return true 如果两个 PoseBinKey 相等
     * @return false 如果两个 PoseBinKey 不相等
     */
    bool operator==(const PoseBinKey & other) const
    {
      return x_bin == other.x_bin && y_bin == other.y_bin && yaw_bin == other.yaw_bin;
    }
  };

  /**
   * @brief 哈希函数，用于在哈希表中索引 PoseBinKey。
   *
   * 该结构体实现了一个自定义的哈希函数，用于将 PoseBinKey 映射到哈希值；哈希可能碰撞，相等性由 operator== 判断。
   */
  struct PoseBinKeyHash
  {
    /**
     * @brief 重载调用运算符，用于计算 PoseBinKey 的哈希值。
     *
     * @param key 要计算哈希值的 PoseBinKey 对象
     * @return std::size_t 哈希值
     */
    std::size_t operator()(const PoseBinKey & key) const;
  };

  /**
   * @brief 将位姿转换为对应的箱子键。
   *
   * @param pose 要转换的位姿
   * @return PoseBinKey 对应的箱子键
   */
  PoseBinKey MakePoseBinKey(const Pose2D & pose) const;

  // Private Members ---------------------------------------------------------*/

  // 线性分箱的大小（单位：米）
  double translation_bin_size_m_;
  // 角度分箱的大小（单位：弧度）
  double yaw_bin_size_rad_;
  // 哈希表，存储箱子键和对应的粒子索引
  std::unordered_map<PoseBinKey, std::size_t, PoseBinKeyHash> bin_id_by_key_;
  // 存储每个粒子所属的箱子索引
  std::vector<std::size_t> bin_id_by_particle_index_;
  // 存储每个箱子中的粒子索引
  std::vector<std::vector<std::size_t>> particle_indices_by_bin_id_;
};

// KdTree was the historical name of this hash-based pose bin index.
using KdTree = PoseBinIndex;

}  // namespace mini_nav_core::localization
