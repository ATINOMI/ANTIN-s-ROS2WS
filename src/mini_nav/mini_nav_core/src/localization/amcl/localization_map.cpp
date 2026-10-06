/**
 * @file localization_map.cpp
 * @brief LocalizationMap 的地图快照、距离场和射线查询实现。
 *
 * 本实现把 Costmap2D 的离散代价转换为定位模块使用的三类状态，并在构造阶段
 * 通过以所有已知占用栅格为源的多源最短路计算障碍物距离。后续激光模型只需查表，
 * 不必在每条射线上重复搜索最近障碍物。
 *
 * @author Antinomy
 * @date 2026-09-21
 */

#include "mini_nav_core/localization/amcl/localization_map.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

namespace mini_nav_core::localization
{

namespace
{

/**
 * @brief 距离场优先队列中的一个候选节点。
 *
 * index 使用与 costs_ 相同的按行一维下标；distance 是从最近已知占用源
 * 累积得到的当前最短距离，单位为米。
 */
struct DistanceEntry
{
  /// 当前候选节点到已知占用源的距离，单位为米。
  double distance;
  /// 候选节点在按行数组中的下标。
  std::size_t index;

  /**
   * @brief 按距离提供反向比较，使 std::greater 形成最小堆。
   * @param other 参与比较的另一候选节点。
   * @return 当前节点距离大于 other 时返回 true。
   */
  bool operator>(const DistanceEntry & other) const
  {
    return distance > other.distance;
  }
};
}  // namespace

/**
 * @brief 从代价地图建立定位快照、自由栅格缓存和障碍物距离场。
 *
 * 构造过程分为三步：复制所有栅格代价并收集已知自由栅格；将每个已知占用栅格
 * 作为距离为零的源压入最小堆；使用八邻域 Dijkstra 松弛其余栅格。距离会被
 * max_obstacle_distance 截断，避免无障碍区域产生无界距离并保持传感器模型的
 * 查询范围稳定。
 *
 * @param costmap 提供地图几何和栅格代价的源代价地图。
 * @param max_obstacle_distance 障碍物距离上限，单位为米，必须为有限正数。
 * @throws std::invalid_argument 当距离上限非法时抛出。
 */
LocalizationMap::LocalizationMap(const Costmap2D & costmap, double max_obstacle_distance)
: size_x_(costmap.GetSizeInCellsX()),                                 // 地图宽度，单位为栅格数
  size_y_(costmap.GetSizeInCellsY()),                                 // 地图高度，单位为栅格数
  resolution_(costmap.GetResolution()),                               // 栅格边长，单位为米
  origin_x_(costmap.GetOriginX()),                                    // 地图左下角世界坐标 x，单位为米
  origin_y_(costmap.GetOriginY()),                                    // 地图左下角世界坐标 y，单位为米
  max_obstacle_distance_(max_obstacle_distance),                      // 障碍物距离上限，单位为米
  costs_(costmap.GetCellCount(), 0),                                  // 栅格代价快照，按行存储
  obstacle_distances_(costmap.GetCellCount(), max_obstacle_distance)  // 障碍物距离场，按行存储
{
  if (!std::isfinite(max_obstacle_distance) || max_obstacle_distance <= 0.0) 
  {
    throw std::invalid_argument("max_obstacle_distance must be finite and greater than zero");
  }

  /*
   * 先复制代价并建立自由栅格列表。free_cells_ 是构造时的快照，后续随机抽样
   * 不需要再次扫描整张地图；未知栅格因代价为 255，不会进入该列表。
   */
  for (unsigned int y = 0; y < size_y_; ++y) 
  {
    for (unsigned int x = 0; x < size_x_; ++x) 
    {
      // 代价地图按行存储，先计算一维下标再访问。
      const std::size_t index = static_cast<std::size_t>(y) * size_x_ + x;

      // 复制代价快照；后续查询不再访问原代价地图。
      costs_[index] = costmap.GetCost(x, y);

      // 收集已知自由栅格；未知栅格不会进入该列表。
      if (IsKnownFree(GridCell{x, y})) 
      {
        free_cells_.push_back(GridCell{x, y});
      }
    }
  }

  /*
   * 所有已知占用栅格同时作为 Dijkstra 的源：距离为零的源越多，队列就越能
   * 从多个方向共同扩展，最终得到每个栅格到最近源的最短八邻域路径代价。
   * 如果地图没有已知占用栅格，距离数组会保留初始化的 max_obstacle_distance_。
   */

  // 优先队列按距离升序排列，最小距离的栅格总是最先被松弛。 
  std::priority_queue<DistanceEntry, std::vector<DistanceEntry>, std::greater<DistanceEntry>> queue;
  for (unsigned int y = 0; y < size_y_; ++y) 
  {
    for (unsigned int x = 0; x < size_x_; ++x) 
    {
      // 从二维坐标计算一维下标。
      const GridCell cell{x, y};
      // 只把已知占用栅格作为源压入队列；未知栅格不会被视为源。
      if (!IsKnownOccupied(cell)) 
      {
        continue;
      }
      // 压入队列的源距离为零。
      const std::size_t index = GetIndex(cell);
      obstacle_distances_[index] = 0.0;
      queue.push(DistanceEntry{0.0, index});
    }
  }

  /**
   * @brief 邻居偏移量结构体。
   * @details 定义了八个邻居的相对坐标和距离乘数。
   */
  struct NeighborOffset
  {
    int dx;  // x 方向的栅格偏移量。
    int dy;  // y 方向的栅格偏移量。
    double distance_multiplier; // 与分辨率相乘得到实际距离的乘数。
  };

  // 八邻域偏移量及对应距离乘数，按顺时针顺序排列。
  constexpr std::array<NeighborOffset, 8> kNeighborOffsets{{
    {1, 0, 1.0}, {-1, 0, 1.0}, {0, 1, 1.0}, {0, -1, 1.0},
    {1,  1, kSqrtTwo}, {1,  -1, kSqrtTwo},
    {-1, 1, kSqrtTwo}, {-1, -1, kSqrtTwo}}};

  /*
   * 这是带有过期条目跳过的标准 Dijkstra 松弛过程。优先队列不支持原地更新，
   * 因此同一栅格获得更短距离时会重新入队；第 78 行的判断会丢弃旧条目。
   */
  // 队列不为空时继续松弛。
  while (!queue.empty()) 
  {
    // 取出当前距离最小的栅格。
    const auto current = queue.top();

    // 弹出队列前检查距离是否过期；如果当前距离大于已知最短距离，说明该条目是旧的。
    queue.pop();

    // 如果当前距离大于已知最短距离，说明该条目是旧的，跳过。
    if (current.distance > obstacle_distances_[current.index]) 
    {
      continue;
    }

    // 从一维下标恢复二维坐标，便于检查边界并遍历八个邻居。
    const unsigned int current_x = static_cast<unsigned int>(current.index % size_x_);
    const unsigned int current_y = static_cast<unsigned int>(current.index / size_x_);
    // 从按行下标恢复二维坐标，便于检查边界并遍历八个邻居。
    for (const auto & direction : kNeighborOffsets) 
    {
      // 计算邻居栅格的坐标。
      const int next_x = static_cast<int>(current_x) + direction.dx;
      const int next_y = static_cast<int>(current_y) + direction.dy;
      // 检查邻居是否越界；越界的邻居不参与松弛。
      if (next_x < 0 || next_y < 0 || 
          next_x >= static_cast<int>(size_x_) ||
          next_y >= static_cast<int>(size_y_)) 
      {
        continue;
      }

      // 计算邻居栅格的一维下标和新的候选距离。
      const double step = resolution_ * direction.distance_multiplier;
      const std::size_t next_index = static_cast<std::size_t>(next_y) * size_x_ + next_x;
      const double next_distance = std::min(max_obstacle_distance_, current.distance + step);
      // 截断只限制保存值，不会把更短的候选距离误判为无效。
      if (next_distance < obstacle_distances_[next_index]) {
        obstacle_distances_[next_index] = next_distance;
        queue.push(DistanceEntry{next_distance, next_index});
      }
    }
  }
}

/**
 * @brief 将世界坐标映射到所在栅格。
 * @param world_x 世界坐标 x，单位为米。
 * @param world_y 世界坐标 y，单位为米。
 * @param cell 输出栅格坐标。
 * @return 坐标有效且位于半开地图区域时返回 true，否则返回 false。
 */
bool LocalizationMap::TryGetCellFromWorld(double world_x, double world_y, GridCell & cell) const
{
  // 检查输入坐标是否有限且位于地图左下角及以上；半开区间的右上边界不包含。
  if (!std::isfinite(world_x) || !std::isfinite(world_y) ||
          world_x < origin_x_ || world_y < origin_y_) 
  {
    return false;
  }

  const double cell_x = (world_x - origin_x_) / resolution_;
  const double cell_y = (world_y - origin_y_) / resolution_;

  // 先检查浮点结果和上界，再转换为无符号下标，避免非法转换或边界溢出。
  if (!std::isfinite(cell_x) || !std::isfinite(cell_y) ||
      cell_x < 0.0 || cell_y < 0.0 ||
      cell_x >= static_cast<double>(size_x_) || cell_y >= static_cast<double>(size_y_)) 
  {
    return false;
  }

  // 将浮点下标转换为无符号整数下标，表示栅格坐标。
  cell = GridCell{
    static_cast<unsigned int>(cell_x),
    static_cast<unsigned int>(cell_y)};
  return true;
}

/**
 * @brief 计算栅格中心的世界坐标。
 * @param cell 已知的地图栅格。
 * @param world_x 输出中心坐标 x，单位为米。
 * @param world_y 输出中心坐标 y，单位为米。
 * @throws std::out_of_range 当栅格超出地图范围时抛出。
 */
void LocalizationMap::GetCellCenter(const GridCell & cell, double & world_x, double & world_y) const
{
  if (!IsInBounds(cell)) 
  {
    throw std::out_of_range("Localization map cell is out of bounds");
  }
  world_x = origin_x_ + (static_cast<double>(cell.x) + 0.5) * resolution_;
  world_y = origin_y_ + (static_cast<double>(cell.y) + 0.5) * resolution_;
}

/**
 * @brief 判断栅格是否为已知自由栅格。
 * @param cell 待查询的栅格。
 * @return 在范围内且代价小于 kLethalObstacle 时返回 true。
 */
bool LocalizationMap::IsKnownFree(const GridCell & cell) const
{
  // 未知代价 kUnknownCost 不满足“小于阈值”，因此不会被视为自由。
  return IsInBounds(cell) && costs_[GetIndex(cell)] < kLethalObstacle;
}

/**
 * @brief 判断栅格是否为已知占用栅格。
 * @param cell 待查询的栅格。
 * @return 在范围内、代价达到阈值且不是未知编码时返回 true。
 */
bool LocalizationMap::IsKnownOccupied(const GridCell & cell) const
{
  return IsInBounds(cell) && costs_[GetIndex(cell)] >= kLethalObstacle &&
         costs_[GetIndex(cell)] != kUnknownCost;
}

/**
 * @brief 判断栅格是否为未知栅格。
 * @param cell 待查询的栅格。
 * @return 在范围内且代价等于 kUnknownCost 时返回 true。
 */
bool LocalizationMap::IsUnknown(const GridCell & cell) const
{
  return IsInBounds(cell) && costs_[GetIndex(cell)] == kUnknownCost;
}

/**
 * @brief 查询世界坐标处的障碍物距离。
 * @param world_x 世界坐标 x，单位为米。
 * @param world_y 世界坐标 y，单位为米。
 * @param distance_m 输出障碍物距离，单位为米。
 * @return 坐标位于地图范围内时返回 true，否则返回 false。
 */
bool LocalizationMap::TryGetObstacleDistanceAtWorld(
  double world_x, double world_y, double & distance_m) const
{
  // 先将世界坐标映射到栅格坐标；失败时不修改 distance_m。
  GridCell cell;
  if (!TryGetCellFromWorld(world_x, world_y, cell)) {
    return false;
  }
  // 栅格坐标有效，返回预计算的障碍物距离。
  distance_m = obstacle_distances_[GetIndex(cell)];
  return true;
}

/**
 * @brief 读取格中心到最近已知障碍物的截断距离。
 *
 * @param cell 图内格下标。
 * @return 障碍距离，米，上限为配置 max_obstacle_distance。
 */
double LocalizationMap::GetObstacleDistanceAtCell(const GridCell & cell) const
{
  if (!IsInBounds(cell)) {
    throw std::out_of_range("Obstacle distance cell is outside the map");
  }
  return obstacle_distances_[GetIndex(cell)];
}

/**
 * @brief 查询障碍物距离，并为越界输入提供距离上限回退值。
 * @param world_x 世界坐标 x，单位为米。
 * @param world_y 世界坐标 y，单位为米。
 * @return 有效栅格的预计算距离；越界或非有限输入返回 max_obstacle_distance_。
 */
double LocalizationMap::GetObstacleDistanceAtWorld(double world_x, double world_y) const
{
  // 越界或非有限输入时返回距离上限，避免异常值传播。
  double distance_m = max_obstacle_distance_;
  // 尝试查询障碍物距离；失败时 distance_m 保持为 max_obstacle_distance_。
  TryGetObstacleDistanceAtWorld(world_x, world_y, distance_m);
  return distance_m;
}

/**
 * @brief 沿位姿朝向采样射线，计算预测的障碍物或地图边界距离。
 * @param ray_pose 射线起点和朝向；位置单位为米，yaw 单位为弧度。
 * @param max_range_m 最大射程，单位为米，必须为有限正数。
 * @param expected_range_m 输出预测距离，单位为米。
 * @return max_range_m 合法并完成查询时返回 true，否则返回 false。
 */
bool LocalizationMap::CastRay(
  const Pose2D & ray_pose, double max_range_m, double & expected_range_m) const
{
  // 检查最大射程是否合法；非法输入时不修改 expected_range_m。
  if (!std::isfinite(max_range_m) || max_range_m <= 0.0) 
  {
    return false;
  }

  /*
   * 采样步长取半个栅格，降低跨过窄障碍物的风险；当分辨率过小时使用
   * kMinimumRayStepM 防止循环步长退化为零或造成异常多的数值迭代。
   */
  // 计算步长，确保不大于半个栅格且不小于最小步长。
  const double step = std::max(resolution_ * kRayStepFraction, kMinimumRayStepM);

  // 沿射线从起点开始采样，直到超过最大射程。
  for (double distance = 0.0; distance <= max_range_m; distance += step) 
  {
    const double x = ray_pose.x + distance * std::cos(ray_pose.yaw);
    const double y = ray_pose.y + distance * std::sin(ray_pose.yaw);
    GridCell cell;
    if (!TryGetCellFromWorld(x, y, cell)) 
    {
      // 射线离开地图即视为命中地图边界，返回当前采样距离。
      expected_range_m = distance;
      return true;
    }
    if (IsKnownOccupied(cell)) 
    {
      // 只有已知占用栅格会终止射线；未知栅格继续作为可穿过区域采样。
      expected_range_m = distance;
      return true;
    }
  }

  // 在最大射程内未发现占用或边界时，按传感器无回波的约定返回最大量程。
  expected_range_m = max_range_m;
  return true;
}

/**
 * @brief 从已知自由栅格缓存中均匀随机抽样。
 * @param generator 用于抽样的伪随机数生成器。
 * @param cell 输出抽到的栅格。
 * @return 缓存非空并完成抽样时返回 true，否则返回 false。
 */
bool LocalizationMap::SampleKnownFreeCell(std::mt19937_64 & generator, GridCell & cell) const
{
  if (free_cells_.empty()) {
    return false;
  }
  std::uniform_int_distribution<std::size_t> distribution(0, free_cells_.size() - 1U);
  cell = free_cells_[distribution(generator)];
  return true;
}

/**
 * @brief 返回已知自由栅格缓存的大小。
 * @return 已知自由栅格数量。
 */
std::size_t LocalizationMap::GetKnownFreeCellCount() const
{
  return free_cells_.size();
}

/** @brief 返回 x 方向的栅格数量。@return x 方向尺寸。 */
unsigned int LocalizationMap::GetSizeInCellsX() const
{
  return size_x_;
}

/** @brief 返回 y 方向的栅格数量。@return y 方向尺寸。 */
unsigned int LocalizationMap::GetSizeInCellsY() const
{
  return size_y_;
}

/** @brief 返回地图分辨率。@return 每个栅格边长，单位为米。 */
double LocalizationMap::GetResolution() const
{
  return resolution_;
}

/** @brief 返回障碍物距离场上限。@return 距离上限，单位为米。 */
double LocalizationMap::GetMaxObstacleDistance() const
{
  return max_obstacle_distance_;
}

/**
 * @brief 将已知栅格坐标转换为按行存储的一维下标。
 * @param cell 已确认在地图范围内的栅格。
 * @return 对应下标，计算公式为 cell.y * size_x_ + cell.x。
 */
std::size_t LocalizationMap::GetIndex(const GridCell & cell) const
{
  return static_cast<std::size_t>(cell.y) * size_x_ + cell.x;
}

/**
 * @brief 判断栅格坐标是否在地图范围内。
 * @param cell 待检查的栅格。
 * @return x < size_x_ 且 y < size_y_ 时返回 true。
 */
bool LocalizationMap::IsInBounds(const GridCell & cell) const
{
  return cell.x < size_x_ && cell.y < size_y_;
}

}  // namespace mini_nav_core::localization
