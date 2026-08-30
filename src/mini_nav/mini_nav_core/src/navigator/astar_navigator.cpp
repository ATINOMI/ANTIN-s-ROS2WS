/* Includes ----------------------------------------------------------------*/
#include "mini_nav_core/navigator/astar_navigator.hpp"
#include <algorithm>
#include <limits>
#include <queue>

using namespace mini_nav_core;

/* Functions Definition --------------------------------------------------------------*/

/**
 * @brief 使用 A* 算法规划路径
 * 
 * @param costmap : 成本地图
 * @param start   : 起点位置
 * @param goal    : 终点位置
 * @return std::vector<MapLocation> : 规划出的路径
 */
std::vector<MapLocation> AStarPlanner::Plan(
  const Costmap2D & costmap,
  const MapLocation & start,
  const MapLocation & goal) const
{
  // 获取地图尺寸
  const unsigned int size_x = costmap.GetSizeInCellsX();
  const unsigned int size_y = costmap.GetSizeInCellsY();

  // 检查起点和终点是否在地图范围内，且不在障碍物上
  if (start.x >= size_x || start.y >= size_y ||
      goal.x  >= size_x || goal.y  >= size_y ||
      costmap.GetCost(start.x, start.y) >= kLethalObstacle ||
      costmap.GetCost(goal.x, goal.y)   >= kLethalObstacle)
  {
    return {};
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
  const unsigned int infinity    = std::numeric_limits<unsigned int>::max();

  //这个数组用于存储从起点到每个节点的实际代价（g 值），初始值为无穷大。
  std::vector<unsigned int> g_score(cell_count, infinity);

  //这个数组用于存储每个节点的父节点索引，以便在找到路径后进行回溯。
  std::vector<unsigned int> parent(cell_count, infinity);

  //这个优先队列用于存储待访问的节点，按照 f 值排序。
  std::priority_queue<OpenNode, 
                      std::vector<OpenNode>, 
                      CompareOpenNode> open_list;

  // 初始化起点的 g 值为 0，并将其加入 open_list。
  g_score[start_index] = 0;
  open_list.push({start_index, ManhattanDistance(start, goal)});

  constexpr int directions[4][2] = {
    { 0,  1},  // Up
    { 1,  0},  // Right
    { 0, -1},  // Down
    {-1,  0}   // Left
  };

    /* A* 搜索循环
    * 1. 从 open_list 中取出 f 值最小的节点作为当前节点。
    * 2. 如果当前节点是目标节点，则搜索结束。
    * 3. 否则，遍历当前节点的四个邻居：
    *    - 如果邻居在地图范围内且不是障碍物，则计算从起点到邻居的 g 值。
    *    - 如果这个 g 值小于邻居当前的 g 值，则更新邻居的 g 值和父节点，并将邻居加入 open_list。
    * 4. 重复步骤 1-3，直到找到目标节点或 open_list 为空（表示没有路径）。              
    */
    while (!open_list.empty()) {

        // 队列出队，获取 open_list 中 f 值最小的节点
        const OpenNode current = open_list.top();
        open_list.pop();
    
        if (current.index == goal_index) break;  // Goal reached
    
        // 计算当前节点的 x 和 y 坐标
        const unsigned int current_x = current.index % size_x;
        const unsigned int current_y = current.index / size_x;
    
        for (const auto & direction : directions) // 遍历四个方向
        {
            // 计算邻居节点的坐标
            const int next_x = static_cast<int>(current_x) + direction[0];
            const int next_y = static_cast<int>(current_y) + direction[1];
        
            // 检查邻居节点是否在地图范围内
            if (next_x < 0 || next_x >= static_cast<int>(size_x) ||
                next_y < 0 || next_y >= static_cast<int>(size_y)) continue;

            // 检查邻居节点是否是障碍物
            if (costmap.GetCost(next_x, next_y) >= kLethalObstacle) continue;

            // 将邻居节点的坐标转换为无符号整数
            const unsigned int mx = static_cast<unsigned int>(next_x);
            const unsigned int my = static_cast<unsigned int>(next_y);

            // 计算邻居节点的索引和从起点到邻居节点的 g 值
            const unsigned int next_index = GetIndex(mx, my, size_x);
            const unsigned int tentative_g_score = g_score[current.index] + 1;

            // 如果新的 g 值不小于邻居节点当前的 g 值，则跳过
            if (tentative_g_score >= g_score[next_index]) continue;
                
            // 更新邻居节点的 g 值和父节点，并将其加入 open_list
            g_score[next_index] = tentative_g_score;
            parent[next_index]  = current.index;
            open_list.push({next_index, 
                            tentative_g_score + ManhattanDistance({mx, my}, 
                            goal)
            });
        }
    }

    if (g_score[goal_index] == infinity) return {};  // No path found
        
    //这里开始路径重建，从目标节点回溯到起点节点，生成最终的路径。
    std::vector<MapLocation> path;
    //使用 current_index 来追踪当前节点的索引，从目标节点开始回溯到起点节点。
    unsigned int current_index = goal_index;
    //当 current_index 不等于起点索引时，继续回溯
    while (current_index != start_index) 
    {
        const unsigned int x = current_index % size_x;
        const unsigned int y = current_index / size_x;
        path.push_back({x, y});
        current_index = parent[current_index];
    }

    // 将起点加入路径，并将路径反转，使其从起点到终点的顺序正确。
    path.push_back(start);
    std::reverse(path.begin(), path.end());
    return path;
}

/**
 * @brief 计算地图上两个位置之间的索引
 * 
 * @param mx 
 * @param my 
 * @param size_x 
 * @return unsigned int 
 */
unsigned int AStarPlanner::GetIndex(
  unsigned int mx,
  unsigned int my,
  unsigned int size_x) const
{
  return my * size_x + mx;
}

/**
 * @brief 计算曼哈顿距离
 *        start 和 goal 之间的距离，作为启发式函数。
 * 
 * @param start : 起点位置
 * @param goal   : 终点位置
 * @return unsigned int 
 */
unsigned int AStarPlanner::ManhattanDistance(
  const MapLocation & start,
  const MapLocation & goal) const
{
  const unsigned int dx = start.x > goal.x ? start.x - goal.x : goal.x - start.x;
  const unsigned int dy = start.y > goal.y ? start.y - goal.y : goal.y - start.y;
  return dx + dy;
}
  // namespace mini_nav_core
