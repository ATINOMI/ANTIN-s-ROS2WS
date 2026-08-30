#pragma once

/* Includes ----------------------------------------------------------------*/

#include <vector>
#include "mini_nav_core/map/costmap_2d.hpp"

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{

/* Structs definition -------------------------------------------------------*/
    struct OpenNode
    {
        unsigned int index;
        unsigned int f_score;
    };

    struct CompareOpenNode
    {
        bool operator()(const OpenNode & left, const OpenNode & right) const
        {
            return left.f_score > right.f_score;
        }
    };

/* Class definition --------------------------------------------------------*/
    class AStarPlanner
    {
    public:
    /**
     * @brief  使用 A* 算法在代价地图上规划从起点到终点的路径。
     * 
     * @param costmap 
     * @param start 
     * @param goal 
     * @return std::vector<MapLocation> 
     */
        std::vector<MapLocation> Plan(
            const Costmap2D   & costmap,
            const MapLocation & start,      
            const MapLocation & goal) const;

    private:
    /**
     * @brief 获取指定栅格在一维数组中的下标。
     * 
     * @param mx 
     * @param my 
     * @param size_x 
     * @return unsigned int 
     */
        unsigned int GetIndex(unsigned int mx, unsigned int my, unsigned int size_x) const;

    /**
     * @brief  计算曼哈顿距离，用于 A* 的启发式函数。
     * 
     * @param from 
     * @param to 
     * @return * unsigned int 
     */
        unsigned int ManhattanDistance(const MapLocation & start, const MapLocation & goal) const;
    };

/* Local constants ---------------------------------------------------------*/
    constexpr unsigned char kLethalObstacle = 254;
}