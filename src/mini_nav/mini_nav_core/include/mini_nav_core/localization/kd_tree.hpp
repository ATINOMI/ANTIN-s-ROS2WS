/**
 * @file kd_tree.hpp
 * @brief KdTree 类的声明，用于在粒子滤波中进行高效的空间索引和查询。
 *
 * 该类实现了一个三维的 Kd 树，用于将粒子根据其位姿（x, y, yaw）进行分箱，以便快速查询和统计。
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

#include "mini_nav_core/localization/types.hpp"

/* Namespace ----------------------------------------------------------------*/

namespace mini_nav_core::localization
{

  /**
   * @class KdTree
   * @brief 三维 Kd 树，用于在粒子滤波中进行高效的空间索引和查询。
   *
   * 该类将粒子根据其位姿（x, y, yaw）进行分箱，以便快速查询和统计。
   * 主要功能包括构建 Kd 树、获取占用的箱子数量等。
   */
class KdTree
{
public:
/* Public Functions --------------------------------------------------------- */

  /**
     * @brief 构造函数，初始化 Kd 树的线性和角度分箱大小。
     *
     * @param linear_bin_size 线性分箱的大小（单位：米）
     * @param angular_bin_size 角度分箱的大小（单位：弧度）
     */
  KdTree(double linear_bin_size = 0.5, double angular_bin_size = 0.261799387799);

  /**
   * @brief 构建 Kd 树，将粒子根据其位姿进行分箱。
   *
   * @param particles 引用的粒子向量，每个粒子包含位姿和权重
   */
  void Build(const std::vector<Particle> & particles);

  /**
   * @brief 获取被占用的格子数量。
   *
   * @return std::size_t 被占用的格子的数量
   */
  std::size_t GetOccupiedBinCount() const;

private:
/* Private Types ------------------------------------------------------------*/


  /**
   * @brief 表示一个三维箱子的键，用于在哈希表中索引。
   *
   * 该结构体包含三个整数，分别表示箱子在 x、y 和 yaw 方向上的索引。
   */
  struct Key
  {
    int x;    //  x 方向的箱子索引
    int y;    //  y 方向的箱子索引
    int yaw;  //  yaw 方向的箱子索引

    /**
     * @brief 重载相等运算符，用于比较两个 Key 是否相等。
     *
     * @param other 另一个 Key 对象
     * @return true 如果两个 Key 相等
     * @return false 如果两个 Key 不相等
     */
    bool operator==(const Key & other) const
    {
      return x == other.x && y == other.y && yaw == other.yaw;
    }
  };

  /**
   * @brief 哈希函数，用于在哈希表中索引 Key。
   *
   * 该结构体实现了一个自定义的哈希函数，用于将 Key 映射到一个唯一的哈希值。
   */
  struct KeyHash
  {
    /**
     * @brief 重载调用运算符，用于计算 Key 的哈希值。
     *
     * @param key 要计算哈希值的 Key 对象
     * @return std::size_t 哈希值
     */
    std::size_t operator()(const Key & key) const;
  };

  /**
   * @brief 将位姿转换为对应的箱子键。
   *
   * @param pose 要转换的位姿
   * @return Key 对应的箱子键
   */
  Key MakeKey(const Pose2D & pose) const;

  // Private Members ---------------------------------------------------------*/

  // 线性分箱的大小（单位：米）
  double linear_bin_size_;
  // 角度分箱的大小（单位：弧度）
  double angular_bin_size_;
  // 哈希表，存储箱子键和对应的粒子索引
  std::unordered_map<Key, std::size_t, KeyHash> bin_ids_;
  // 存储每个粒子所属的箱子索引
  std::vector<std::size_t> particle_bins_;
  // 存储每个箱子中的粒子索引
  std::vector<std::vector<std::size_t>> bin_particles_;
};

}  // namespace mini_nav_core::localization
