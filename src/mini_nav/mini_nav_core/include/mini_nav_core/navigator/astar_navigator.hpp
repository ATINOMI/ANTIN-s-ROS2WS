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
        double f_score;
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
        /// 软代价相对于一步自由栅格距离的权重，必须为有限非负数。
        explicit AStarPlanner(double cost_travel_multiplier = 2.0);

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

        /** 在规划图上搜索，并用原图逐边检查圆形车体的连续扫掠。 */
        std::vector<MapLocation> Plan(
            const Costmap2D & planning,
            const Costmap2D & source,
            double clearance_radius,
            const MapLocation & start,
            const MapLocation & goal,
            bool include_unknown_clearance = true) const;

    private:
        double cost_travel_multiplier_;
        std::vector<MapLocation> PlanImpl(
            const Costmap2D & planning,
            const Costmap2D * source,
            double clearance_radius,
            const MapLocation & start,
            const MapLocation & goal,
            bool include_unknown_clearance) const;
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
     * @brief 计算允许八邻域移动时的八方向距离启发式。
     * 
     * @param from 
     * @param to 
     * @return 不超过自由空间最短路径代价的距离（格）。
     */
        double OctileDistance(const MapLocation & start, const MapLocation & goal) const;
    };

/* Local constants ---------------------------------------------------------*/
    constexpr unsigned char kInscribedInflatedObstacle = 253;
    constexpr unsigned char kLethalObstacle = 254;
}
