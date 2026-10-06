/**
 * @file localization_map.hpp
 * @brief 与 ROS 解耦的定位地图快照及查询接口。
 *
 * LocalizationMap 从 Costmap2D 复制地图几何信息和栅格代价，并预先缓存已知
 * 自由栅格与到已知障碍物的距离。定位算法因此可以在不持有或修改原始代价地图
 * 的情况下完成坐标转换、激光射线查询和全局粒子初始化。
 *
 * 本文件约定 x/y 使用米，栅格坐标使用从 0 开始的无符号索引，yaw 使用弧度。
 * 已知自由、已知占用和未知是互斥的地图语义；未知栅格不会被作为全局初始化
 * 的自由位置，也不会被 CastRay() 当作障碍物命中。
 *
 * @author Antinomy
 * @date 2026-09-21
 */
#pragma once

#include <cstddef>
#include <random>
#include <vector>

#include "mini_nav_core/localization/amcl/localization_constants.hpp"
#include "mini_nav_core/localization/amcl/types.hpp"
#include "mini_nav_core/map/costmap_2d.hpp"

namespace mini_nav_core::localization
{

/**
 * @brief 表示一个二维栅格坐标。
 *
 * x 和 y 是从零开始的栅格下标，而不是以米表示的世界坐标。只有满足
 * x < GetSizeInCellsX() 且 y < GetSizeInCellsY() 的坐标才位于地图内部。
 */
struct GridCell
{
  /// x 方向的栅格下标。
  unsigned int x{0};
  /// y 方向的栅格下标。
  unsigned int y{0};
};

/// 兼容旧调用方的名称别名；字段仍表示栅格下标而非世界坐标。
using MapCell = GridCell;

/**
 * @class LocalizationMap
 * @brief 提供定位所需的只读地图快照和空间查询。
 *
 * 构造时会复制 Costmap2D 的栅格代价，并建立以下派生数据：已知自由栅格列表
 * 以及每个栅格到最近已知占用栅格的距离。对象构造完成后不再依赖源 Costmap2D，
 * 因而源地图后续变化不会自动反映到本对象；需要新快照时应重新构造。
 *
 * 栅格代价采用以下分类：小于 kLethalObstacle 为已知自由，达到该阈值且不等于
 * kUnknownCost 为已知占用，等于 kUnknownCost 为未知。未知栅格会保留其未知语义，
 * 但障碍物距离传播可以经过它，因为距离场描述的是几何上的最近已知障碍物。
 */
class LocalizationMap
{
public:
  /// 已知占用的最低代价阈值；该值本身也属于已知占用。
  static constexpr unsigned char kLethalObstacle = 254;
  /// 未知栅格的代价编码；未知不等同于已知占用或已知自由。
  static constexpr unsigned char kUnknownCost = 255;

  /**
   * @brief 从代价地图建立定位地图快照及障碍物距离场。
   *
   * @param costmap 提供地图几何、代价和栅格数量的源代价地图；只读访问。
   * @param max_obstacle_distance 距离场的上限，单位为米；未能在上限内找到障碍物
   *                              的栅格也返回该值。
   * @throws std::invalid_argument 当 max_obstacle_distance 不是有限正数时抛出。
   * @note 距离场使用八邻域步长，直行代价为 resolution，斜行代价为
   *       resolution × sqrt(2)。
   */
  explicit LocalizationMap(
    const Costmap2D & costmap,
    double max_obstacle_distance = kDefaultMaxObstacleDistanceM);

  /**
   * @brief 将世界坐标转换为所在栅格。
   *
   * 世界坐标以地图原点为基准按分辨率取栅格下标；地图左边界和下边界包含，
   * 右边界和上边界不包含。失败时不修改 cell。
   *
   * @param world_x 世界坐标 x，单位为米。
   * @param world_y 世界坐标 y，单位为米。
   * @param cell 返回所在栅格。
   * @return 坐标有限且位于地图范围内时返回 true，否则返回 false。
   */
  bool TryGetCellFromWorld(double world_x, double world_y, GridCell & cell) const;

  /**
   * @brief 返回栅格中心的世界坐标。
   *
   * 栅格 (x, y) 的中心为原点加上 (x + 0.5, y + 0.5) 个分辨率；这使全局
   * 粒子初始化和栅格地图查询使用同一空间约定。
   *
   * @param cell 待转换的地图栅格。
   * @param world_x 返回中心世界坐标 x，单位为米。
   * @param world_y 返回中心世界坐标 y，单位为米。
   * @throws std::out_of_range 当 cell 不在地图范围内时抛出。
   */
  void GetCellCenter(const GridCell & cell, double & world_x, double & world_y) const;

  /**
   * @brief 判断栅格是否为已知自由栅格。
   * @param cell 待查询的栅格坐标。
   * @return 坐标在范围内且代价小于 kLethalObstacle 时返回 true。
   * @note 未知代价 kUnknownCost 不满足“小于阈值”，因此不会被视为自由。
   */
  bool IsKnownFree(const GridCell & cell) const;

  /**
   * @brief 判断栅格是否为已知占用栅格。
   * @param cell 待查询的栅格坐标。
   * @return 坐标在范围内、代价达到阈值且不为未知编码时返回 true。
   */
  bool IsKnownOccupied(const GridCell & cell) const;

  /**
   * @brief 判断栅格是否为未知栅格。
   * @param cell 待查询的栅格坐标。
   * @return 坐标在范围内且代价等于 kUnknownCost 时返回 true。
   */
  bool IsUnknown(const GridCell & cell) const;

  /** @brief 兼容旧接口，等价于 IsKnownFree()。@param cell 待查询的栅格。 */
  bool IsFree(const GridCell & cell) const { return IsKnownFree(cell); }
  /** @brief 兼容旧接口，等价于 IsKnownOccupied()。@param cell 待查询的栅格。 */
  bool IsOccupied(const GridCell & cell) const { return IsKnownOccupied(cell); }

  /**
   * @brief 查询世界坐标所在栅格的障碍物距离。
   * @param world_x 世界坐标 x，单位为米。
   * @param world_y 世界坐标 y，单位为米。
   * @param distance_m 返回到最近已知占用栅格的距离，单位为米。
   * @return 坐标位于地图范围内时返回 true，否则返回 false；失败时不修改 distance_m。
   */
  bool TryGetObstacleDistanceAtWorld(
    double world_x, double world_y, double & distance_m) const;

  /**
   * @brief 查询指定栅格已缓存的障碍物距离。
   * @param cell 栅格坐标。
   * @return 到最近已知障碍物的距离，单位为米。
   * @throws std::out_of_range 当 cell 不在地图范围内时抛出。
   */
  double GetObstacleDistanceAtCell(const GridCell & cell) const;

  /**
   * @brief 查询障碍物距离并以距离上限作为越界回退值。
   * @param world_x 世界坐标 x，单位为米。
   * @param world_y 世界坐标 y，单位为米。
   * @return 栅格距离或 max_obstacle_distance；世界坐标越界、非有限时返回后者。
   * @note 该回退行为用于兼容旧调用方；需要区分越界和有效距离时使用
   *       TryGetObstacleDistanceAtWorld()。
   */
  double GetObstacleDistanceAtWorld(double world_x, double world_y) const;

  /**
   * @brief 沿给定二维位姿的朝向离散投射射线。
   *
   * 从 ray_pose 的位置开始，以不大于半个栅格且不小于最小步长的距离采样。
   * 遇到已知占用栅格或离开地图时返回对应采样距离；直到 max_range_m 都未命中
   * 时返回 max_range_m。未知栅格不会直接终止射线。
   *
   * @param ray_pose 射线起点及朝向；位置单位为米，yaw 单位为弧度。
   * @param max_range_m 最大射程，单位为米，必须为有限正数。
   * @param expected_range_m 返回预测射程，单位为米。
   * @return max_range_m 合法且射线完成查询时返回 true，否则返回 false。
   */
  bool CastRay(
    const Pose2D & ray_pose, double max_range_m, double & expected_range_m) const;

  /**
   * @brief 从预缓存的已知自由栅格中均匀随机抽取一个栅格。
   * @param generator 用于抽样的伪随机数生成器。
   * @param cell 返回抽取到的已知自由栅格。
   * @return 存在已知自由栅格且抽样成功时返回 true，否则返回 false。
   */
  bool SampleKnownFreeCell(std::mt19937_64 & generator, GridCell & cell) const;

  /**
   * @brief 返回已知自由栅格数量。
   * @return 构造快照时识别出的已知自由栅格数量。
   */
  std::size_t GetKnownFreeCellCount() const;

  /**
   * @brief 兼容旧接口，等价于 SampleKnownFreeCell()。
   * @param generator 用于抽样的伪随机数生成器。
   * @param cell 返回抽取到的自由栅格。
   * @return 抽样成功时返回 true，否则返回 false。
   */
  bool SampleFreeCell(std::mt19937_64 & generator, GridCell & cell) const
  {
    return SampleKnownFreeCell(generator, cell);
  }

  /**
   * @brief 兼容旧接口，等价于 GetKnownFreeCellCount()。
   * @return 已知自由栅格数量。
   */
  std::size_t GetFreeCellCount() const { return GetKnownFreeCellCount(); }

  /** @brief 返回 x 方向的栅格数量。@return x 方向尺寸。 */
  unsigned int GetSizeInCellsX() const;
  /** @brief 返回 y 方向的栅格数量。@return y 方向尺寸。 */
  unsigned int GetSizeInCellsY() const;
  /** @brief 返回栅格分辨率。@return 每个栅格边长，单位为米。 */
  double GetResolution() const;
  /** @brief 返回障碍物距离场的上限。@return 距离上限，单位为米。 */
  double GetMaxObstacleDistance() const;

private:
  /**
   * @brief 将栅格坐标转换为按行存储数组的一维下标。
   * @param cell 已确认在地图范围内的栅格坐标。
   * @return 对应下标，计算公式为 cell.y * size_x_ + cell.x。
   * @note 调用方必须先完成范围检查；本函数本身不检查边界。
   */
  std::size_t GetIndex(const GridCell & cell) const;

  /**
   * @brief 判断栅格坐标是否位于当前地图范围内。
   * @param cell 待检查的栅格坐标。
   * @return x < size_x_ 且 y < size_y_ 时返回 true。
   */
  bool IsInBounds(const GridCell & cell) const;

  /// 地图 x 方向栅格数量。
  unsigned int size_x_{0};
  /// 地图 y 方向栅格数量。
  unsigned int size_y_{0};
  /// 每个栅格边长，单位为米。
  double resolution_{0.0};
  /// 地图左下角原点的世界坐标 x，单位为米。
  double origin_x_{0.0};
  /// 地图左下角原点的世界坐标 y，单位为米。
  double origin_y_{0.0};
  /// 障碍物距离场的最大保存值，单位为米。
  double max_obstacle_distance_{0.0};
  /// 从源代价地图复制的按行栅格代价。
  std::vector<unsigned char> costs_;
  /// 每个栅格到最近已知占用栅格的距离，按行与 costs_ 对齐，单位为米。
  std::vector<double> obstacle_distances_;
  /// 构造时缓存的已知自由栅格，用于全局初始化的均匀抽样。
  std::vector<GridCell> free_cells_;
};

}  // namespace mini_nav_core::localization
