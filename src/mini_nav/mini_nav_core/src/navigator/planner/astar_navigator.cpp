/**
 * @file astar_navigator.cpp
 * @brief 八邻域 A*、软代价与安全终点容差规划。
 * @author Antinomy
 * @date 2026-10-01
 */
/* Includes ----------------------------------------------------------------*/
#include "mini_nav_core/navigator/planner/astar_navigator.hpp"
#include "mini_nav_core/collision_checker/collision_checker.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <stdexcept>

using namespace mini_nav_core;

/**
 * @brief 设置 A* 的软代价权重。
 *
 * @param cost_travel_multiplier 每步软代价相对于几何距离的权重，有限非负。
 * @throws std::invalid_argument 权重为负或非有限。
 */
AStarPlanner::AStarPlanner(double cost_travel_multiplier) : cost_travel_multiplier_(cost_travel_multiplier)
{
  if (!std::isfinite(cost_travel_multiplier_) || cost_travel_multiplier_ < 0.0) 
  {
    throw std::invalid_argument("cost_travel_multiplier must be finite and nonnegative");
  }
}

/* Functions Definition --------------------------------------------------------------*/

/**
 * @brief 在八邻域代价图上规划路径，并在原目标不可达时搜索容差内终点。
 *
 * 优先到达原目标；后备终点只从实际展开的可达格中选，先比较到原目标的欧氏距离，
 * 同距离再比较累计代价。本重载不执行原始地图上的完整车体扫掠。
 *
 * @param costmap 规划代价图，253、254、255 均禁行。
 * @param start 起点栅格下标。
 * @param goal 原目标栅格下标，越界仍拒绝。
 * @param goal_tolerance 原目标不可达时允许的替代终点半径，米；0 禁用。
 * @return 包含起点和最终终点的栅格序列；起点不安全、目标越界或无路时为空。
 * @throws std::invalid_argument 目标容差为负或非有限。
 */
std::vector<MapLocation> AStarPlanner::Plan(
  const Costmap2D & costmap,
  const MapLocation & start,
  const MapLocation & goal,
  double goal_tolerance) const
{
  return PlanImpl(costmap, nullptr, 0.0, start, goal, true, goal_tolerance);
}

/**
 * @brief 执行八邻域 A*，同时在原图上检查每条边的连续车体扫掠。
 *
 * 不允许斜向穿过任一侧禁行格；原目标不可达时仅放宽终点位置，不放宽安全规则。
 *
 * @param planning                  膨胀规划图，253 及以上禁行。
 * @param source                    与规划图几何一致的原始碰撞地图。
 * @param clearance_radius          车体外接圆加安全余量，有限正数，米。
 * @param start                     起点栅格下标。
 * @param goal                      原目标栅格下标，越界仍拒绝。
 * @param goal_tolerance            原目标不可达时允许的替代终点半径，米；0 禁用。
 * @param include_unknown_clearance 是否检查车体与未知格的连续余量。
 * @return 安全栅格路径；无安全可达终点时为空。
 * @throws std::invalid_argument 地图不匹配、安全半径或容差非法。
 */
std::vector<MapLocation> AStarPlanner::Plan(
  const Costmap2D & planning,
  const Costmap2D & source,
  double clearance_radius,
  const MapLocation & start,
  const MapLocation & goal,
  bool include_unknown_clearance,
  double goal_tolerance) const
{
  if (!std::isfinite(clearance_radius) ||                       // 检查安全半径是否为有限数
      clearance_radius <= 0.0 ||                                // 检查安全半径是否为正数
      planning.GetSizeInCellsX() != source.GetSizeInCellsX() || // 检查规划图和原图的x尺寸是否匹配
      planning.GetSizeInCellsY() != source.GetSizeInCellsY() || // 检查规划图和原图的y尺寸是否匹配
      planning.GetResolution() != source.GetResolution() ||     // 检查规划图和原图的分辨率是否匹配
      planning.GetOriginX() != source.GetOriginX() ||           // 检查规划图和原图的原点x坐标是否匹配
      planning.GetOriginY() != source.GetOriginY())             // 检查规划图和原图的原点y坐标是否匹配
  {
    throw std::invalid_argument("A* continuous clearance needs matching maps and a positive radius");
  }

  return PlanImpl(planning, 
                  &source, 
                  clearance_radius, 
                  start, 
                  goal,
                  include_unknown_clearance, 
                  goal_tolerance);
}

/** 使用同一原始几何检查实际起点连接及每条搜索边。 */
std::vector<MapLocation> AStarPlanner::Plan(
    const Costmap2D & planning, const CollisionGeometry & geometry,
    const PathPoint & actual_start, const MapLocation & start,
    const MapLocation & goal, double goal_tolerance,
    const CollisionGeometry * terminal_geometry,
    const CollisionGeometry * start_geometry) const
{
    const auto & source = geometry.grid;
    if (planning.GetSizeInCellsX() != source.GetSizeInCellsX() ||
        planning.GetSizeInCellsY() != source.GetSizeInCellsY() ||
        planning.GetResolution() != source.GetResolution() ||
        planning.GetOriginX() != source.GetOriginX() ||
        planning.GetOriginY() != source.GetOriginY()) {
        throw std::invalid_argument("Collision geometry must match planning map");
    }
    if (!source.IsInBounds(start.x, start.y)) return {};
    if (!geometry.IsClear(actual_start, actual_start) ||
        (start_geometry && !start_geometry->IsClear(actual_start, actual_start))) return {};
    return PlanImpl(planning, &source, geometry.radius, start, goal,
                    geometry.include_unknown, goal_tolerance, &geometry, terminal_geometry,
                    &actual_start, start_geometry);
}

/**
 * @brief 维护最小 f 值队列、累计代价及父索引，搜索后回溯最终路径。
 *
 * 步长为 1 或 √2，乘以 (1 + 权重 × 目标格代价 / 252)。
 * 非负软代价使八方向几何启发式不高估；目标原格优先于容差候选。
 *
 * @param costmap                   膨胀规划图，253 及以上禁行。
 * @param source                    原图指针；nullptr 时跳过连续扫掠。
 * @param clearance_radius          扫掠半径，米。
 * @param start                     起点下标。
 * @param goal                      原目标下标。
 * @param include_unknown_clearance 未知格余量检查开关。
 * @param goal_tolerance            替代终点半径，米。
 * @return 起点至选中终点的路径，无路返回空。
 * @throws std::invalid_argument 容差非法。
 */
std::vector<MapLocation> AStarPlanner::PlanImpl(
  const Costmap2D & costmap,
  const Costmap2D * source,
  double clearance_radius,
  const MapLocation & start,
  const MapLocation & goal,
  bool include_unknown_clearance,
  double goal_tolerance, const CollisionGeometry * geometry,
  const CollisionGeometry * terminal_geometry, const PathPoint * actual_start,
  const CollisionGeometry * start_geometry) const
{
  // 检查 goal_tolerance 是否为有限非负数
  if (!std::isfinite(goal_tolerance) || goal_tolerance < 0.0) 
  {
    throw std::invalid_argument("goal_tolerance must be finite and nonnegative");
  }
  // 获取地图尺寸
  const unsigned int size_x = costmap.GetSizeInCellsX();
  const unsigned int size_y = costmap.GetSizeInCellsY();
  const auto blocked = [&](unsigned int x, unsigned int y) {
    if (geometry) {
      PathPoint point{};
      geometry->grid.MapToWorld(x, y, point.x, point.y);
      return !geometry->IsClear(point, point);
    }
    return costmap.GetCost(x, y) >= kInscribedInflatedObstacle;
  };

  // 检查起点和终点是否在地图范围内，且不在障碍物上
  if (start.x >= size_x || start.y >= size_y ||                                                 // 检查起点是否越界
      goal.x  >= size_x || goal.y  >= size_y ||                                                 // 检查终点是否越界
      (!actual_start && blocked(start.x, start.y)) ||    // 检查起点是否在障碍物上
      (goal_tolerance == 0.0 && blocked(goal.x, goal.y))) // 检查终点是否在障碍物上（仅当 goal_tolerance 为 0 时检查）
  {
    return {};
  }

  if (source != nullptr && geometry == nullptr)
  {
    PathPoint start_point{};  // 起点的世界坐标
    PathPoint goal_point{};   // 终点的世界坐标
    source->MapToWorld(start.x, start.y, start_point.x, start_point.y); // 将起点栅格坐标转换为世界坐标
    source->MapToWorld(goal.x, goal.y, goal_point.x, goal_point.y);     // 将终点栅格坐标转换为世界坐标

    // 检查起点和终点的圆形扫掠是否安全
    if (!IsCircularSweepClear(*source, start_point, start_point, clearance_radius, include_unknown_clearance) ||
        (goal_tolerance == 0.0 && !IsCircularSweepClear(*source, goal_point, goal_point, clearance_radius, include_unknown_clearance)))
    {
      return {};
    }
  }

   /*
    * -cell_count:  地图总栅格数。
    * -start_index: 起点的索引。
    * -goal_index:  目标点的索引。
    * -infinity:    无穷大值，用于初始化 g_score。
    */
  const unsigned int cell_count  = size_x * size_y;
  const unsigned int start_index = GetIndex(start.x, start.y, size_x);
  const unsigned int goal_index  = GetIndex(goal.x, goal.y, size_x);
  const unsigned int invalid_parent = std::numeric_limits<unsigned int>::max();
  const double infinity = std::numeric_limits<double>::infinity();

  //这个数组用于存储从起点到每个节点的实际代价（g 值），初始值为无穷大。
  std::vector<double> g_score(cell_count, infinity);

  //这个数组用于存储每个节点的父节点索引，以便在找到路径后进行回溯。
  std::vector<unsigned int> parent(cell_count, invalid_parent);

  // 八方向启发式一致，每格只展开一次；后备目标仅从实际到达的安全格中选。
  // 这个数组用于标记每个节点是否已经被扩展过，初始值为 false。
  std::vector<bool> expanded(cell_count, false);

  // 这个变量用于存储最终路径的终点索引，初始值为 invalid_parent。
  unsigned int end_index = invalid_parent;

  // 这个变量用于存储最近的可达安全格的索引，初始值为 invalid_parent。
  unsigned int nearest_index = invalid_parent;

  // 这个变量用于存储最近的可达安全格的距离，初始值为 infinity。
  double nearest_distance = infinity;

  //这个优先队列用于存储待访问的节点，按照 f 值排序。
  std::priority_queue<OpenNode, std::vector<OpenNode>, CompareOpenNode> open_list;

  // 初始化起点的 g 值为 0，并将其加入 open_list。
  if (!actual_start) {
    g_score[start_index] = 0;
    open_list.push({start_index, OctileDistance(start, goal)});
  } else {
    const auto add_anchor = [&](unsigned int x, unsigned int y) {
      if (blocked(x, y)) return false;
      PathPoint point{};
      costmap.MapToWorld(x, y, point.x, point.y);
      if (!geometry->IsClear(*actual_start, point) ||
          (start_geometry && !start_geometry->IsClear(*actual_start, point))) return false;
      const unsigned int index = GetIndex(x, y, size_x);
      g_score[index] = std::hypot(point.x - actual_start->x, point.y - actual_start->y) /
          costmap.GetResolution() * (1.0 + cost_travel_multiplier_ * costmap.GetCost(x, y) / 252.0);
      open_list.push({index, g_score[index] + OctileDistance({x, y}, goal)});
      return true;
    };
    // 所属格中心可连通时保持原入口；否则以全部安全邻接中心作为搜索根。
    if (!add_anchor(start.x, start.y)) {
      for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
        if (dx == 0 && dy == 0) continue;
        const int x = static_cast<int>(start.x) + dx;
        const int y = static_cast<int>(start.y) + dy;
        if (x >= 0 && y >= 0 && x < static_cast<int>(size_x) && y < static_cast<int>(size_y))
          add_anchor(static_cast<unsigned int>(x), static_cast<unsigned int>(y));
      }
    }
  }

  constexpr int directions[8][2] = {
    { 0,  1},  // Up
    { 1,  0},  // Right
    { 0, -1},  // Down
    {-1,  0},  // Left
    { 1,  1},  // Up-Right
    { 1, -1},  // Down-Right
    {-1, -1},  // Down-Left
    {-1,  1}   // Up-Left
  };

    /* A* 搜索循环
    * 1. 从 open_list 中取出 f 值最小的节点作为当前节点。
    * 2. 如果当前节点是目标节点，则搜索结束。
    * 3. 否则，遍历当前节点的八个邻居：
    *    - 如果邻居在地图范围内且不是障碍物，则计算从起点到邻居的 g 值。
    *    - 如果这个 g 值小于邻居当前的 g 值，则更新邻居的 g 值和父节点，并将邻居加入 open_list。
    * 4. 重复步骤 1-3，直到找到目标节点或 open_list 为空（表示没有路径）。              
    */
    while (!open_list.empty()) {

        // 队列出队，获取 open_list 中 f 值最小的节点
        const OpenNode current = open_list.top();
        open_list.pop();
    
        // 如果当前节点已经被扩展过，则跳过
        if (expanded[current.index]) continue;

        // 标记当前节点为已扩展
        expanded[current.index] = true;

        PathPoint terminal_point{};
        costmap.MapToWorld(current.index % size_x, current.index / size_x,
            terminal_point.x, terminal_point.y);
        // 路线可暂不融合动态点，但停车点必须通过当前完整几何检查。
        const bool terminal_clear = !terminal_geometry ||
            terminal_geometry->IsClear(terminal_point, terminal_point);
        // 如果当前节点是目标节点，则搜索结束
        if (current.index == goal_index && terminal_clear)
        {
            end_index = goal_index;
            break;  // Goal reached
        }
    
        // 计算当前节点的 x 和 y 坐标
        const unsigned int current_x = current.index % size_x;
        const unsigned int current_y = current.index / size_x;

        // 计算当前节点到目标节点的欧氏距离，并检查是否在 goal_tolerance 范围内
        const double distance = std::hypot( static_cast<double>(current_x) - goal.x,
                                            static_cast<double>(current_y) - goal.y) * costmap.GetResolution();

        if (terminal_clear && goal_tolerance > 0.0 &&           // 启用 goal_tolerance
            distance <= goal_tolerance &&     // 当前节点在容差范围内
            (distance < nearest_distance ||   // 当前节点比最近的可达安全格更近
             (distance == nearest_distance && g_score[current.index] < g_score[nearest_index]))) // 当前节点与最近的可达安全格距离相等,但是代价更小
        {
            // 更新最近的可达安全格
            nearest_index = current.index;
            nearest_distance = distance;
        }
    
        for (const auto & direction : directions) // 遍历八个方向
        {
            // 计算邻居节点的坐标
            const int next_x = static_cast<int>(current_x) + direction[0];
            const int next_y = static_cast<int>(current_y) + direction[1];
        
            // 检查邻居节点是否在地图范围内
            if (next_x < 0 || next_x >= static_cast<int>(size_x) ||
                next_y < 0 || next_y >= static_cast<int>(size_y)) continue;

            // 检查邻居节点是否是障碍物
            const auto cell_cost = costmap.GetCost(next_x, next_y);
            if (blocked(next_x, next_y)) continue;

            // 斜向移动会扫过两侧相邻格；任一侧为禁行区时不能穿角。
            const bool diagonal = direction[0] != 0 && direction[1] != 0; // 判断是否为斜向移动

            if (diagonal &&  // 是否斜向移动
                (blocked(next_x, current_y) || // 检查水平邻居是否为障碍物
                 blocked(current_x, next_y)))  // 检查垂直邻居是否为障碍物
            {
                continue;
            }

            // 将邻居节点的坐标转换为无符号整数
            const unsigned int mx = static_cast<unsigned int>(next_x);
            const unsigned int my = static_cast<unsigned int>(next_y);

            // 计算邻居节点的索引和从起点到邻居节点的 g 值
            const unsigned int next_index = GetIndex(mx, my, size_x);
            // 计算步长和步长代价，步长为 1 或 √2
            const double step_length = diagonal ? std::sqrt(2.0) : 1.0;
            // 计算步长代价，考虑软代价权重和邻居节点的代价值
            const double step_cost = step_length * (1.0 + cost_travel_multiplier_ * static_cast<double>(cell_cost) / 252.0);
            const double tentative_g_score = g_score[current.index] + step_cost;

            // 如果新的 g 值不小于邻居节点当前的 g 值，则跳过
            if (tentative_g_score >= g_score[next_index]) continue;

            // 直接验证整条相邻中心线的车体圆盘；因此膨胀图无须额外半格余量。
            if (source != nullptr) 
            {
                PathPoint current_point{};
                PathPoint next_point{};
                source->MapToWorld(current_x, current_y, current_point.x, current_point.y);
                source->MapToWorld(mx, my, next_point.x, next_point.y);
                
                if (!(geometry ? geometry->IsClear(current_point, next_point) :
                      IsCircularSweepClear(*source, current_point, next_point, clearance_radius,
                                          include_unknown_clearance)))
                {
                    continue;
                }

            }
                
            // 更新邻居节点的 g 值和父节点，并将其加入 open_list
            g_score[next_index] = tentative_g_score;
            parent[next_index]  = current.index;
            open_list.push({next_index, 
                            tentative_g_score + OctileDistance({mx, my},
                            goal)
            });
        }
    }

    // 原目标未到达时须搜索完可达区域，保证选到容差内最近的点。
    if (end_index == invalid_parent) end_index = nearest_index;
    if (end_index == invalid_parent) return {};  // No path found
        
    //这里开始路径重建，从目标节点回溯到起点节点，生成最终的路径。
    std::vector<MapLocation> path;
    //使用 current_index 来追踪当前节点的索引，从目标节点开始回溯到起点节点。
    unsigned int current_index = end_index;
    //当 current_index 还有父节点时，继续回溯至安全起点锚点
    while (parent[current_index] != invalid_parent)
    {
        const unsigned int x = current_index % size_x;
        const unsigned int y = current_index / size_x;
        path.push_back({x, y});
        current_index = parent[current_index];
    }

    // 将起点加入路径，并将路径反转，使其从起点到终点的顺序正确。
    path.push_back({current_index % size_x, current_index / size_x});
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * @brief 把栅格下标映射到按行存储的一维索引。
 *
 * @param mx x 下标。
 * @param my y 下标。
 * @param size_x 每行格数。
 * @return my × size_x + mx；本函数不检查边界。
 */
unsigned int AStarPlanner::GetIndex(
  unsigned int mx,
  unsigned int my,
  unsigned int size_x) const
{
  return my * size_x + mx;
}

/**
 * @brief 计算八邻域几何启发式，不计软代价。
 *
 * @param start 起点下标。
 * @param goal 目标下标。
 * @return max(dx,dy) + (√2 - 1) × min(dx,dy)，单位格。
 */
double AStarPlanner::OctileDistance(
  const MapLocation & start,
  const MapLocation & goal) const
{
  const unsigned int dx = start.x > goal.x ? start.x - goal.x : goal.x - start.x;
  const unsigned int dy = start.y > goal.y ? start.y - goal.y : goal.y - start.y;
  const unsigned int diagonal = std::min(dx, dy);
  return static_cast<double>(dx + dy - 2 * diagonal) +
    std::sqrt(2.0) * static_cast<double>(diagonal);
}
  // namespace mini_nav_core
