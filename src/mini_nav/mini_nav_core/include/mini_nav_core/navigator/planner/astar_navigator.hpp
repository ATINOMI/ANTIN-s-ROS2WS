/**
 * @file astar_navigator.hpp
 * @brief 八邻域 A*、软代价与安全终点容差规划。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

/* Includes ----------------------------------------------------------------*/

#include <vector>
#include "mini_nav_core/map/costmap_2d.hpp"
#include "mini_nav_core/collision_checker/collision_checker.hpp"

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{

/* Structs definition -------------------------------------------------------*/
    /**
     * @brief A* 开放队列条目，保存线性格索引和 f=g+h 优先级。
     */
    struct OpenNode
    {
        unsigned int index;
        double f_score;
    };

    /**
     * @brief 使标准最大堆按最小 f 值优先出队的比较器。
     */
    struct CompareOpenNode
    {
        /**
         * @brief 比较开放队列条目的 f 值，使较小者优先。
         * @param left 左条目。
         * @param right 右条目。
         * @return left 的 f 较大时为 true；相等时 false。
         */
        bool operator()(const OpenNode & left, const OpenNode & right) const
        {
            return left.f_score > right.f_score;
        }
    };

/* Class definition --------------------------------------------------------*/
    /**
     * @brief 在膨胀图上进行八邻域软代价规划，并可选验证完整车体扫掠。
     */
    class AStarPlanner
    {
    public:
        /// 软代价相对于一步自由栅格距离的权重，必须为有限非负数。
        /**
         * @brief 设置 A* 的软代价权重。
         *
         * @param cost_travel_multiplier 每步软代价相对于几何距离的权重，有限非负。
         * @throws std::invalid_argument 权重为负或非有限。
         */
        explicit AStarPlanner(double cost_travel_multiplier = 2.0);

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
        std::vector<MapLocation> Plan(
            const Costmap2D   & costmap,
            const MapLocation & start,      
            const MapLocation & goal,
            double goal_tolerance = 0.0) const;

        /** 在规划图上搜索，并用原图逐边检查圆形车体的连续扫掠。 */
        // 原目标不可达时，在其格中心 goal_tolerance 米内选最近的可达安全格；0 禁用。
        /**
         * @brief 执行八邻域 A*，同时在原图上检查每条边的连续车体扫掠。
         *
         * 不允许斜向穿过任一侧禁行格；原目标不可达时仅放宽终点位置，不放宽安全规则。
         *
         * @param planning 膨胀规划图，253 及以上禁行。
         * @param source 与规划图几何一致的原始碰撞地图。
         * @param clearance_radius 车体外接圆加安全余量，有限正数，米。
         * @param start 起点栅格下标。
         * @param goal 原目标栅格下标，越界仍拒绝。
         * @param goal_tolerance 原目标不可达时允许的替代终点半径，米；0 禁用。
         * @param include_unknown_clearance 是否检查车体与未知格的连续余量。
         * @return 安全栅格路径；无安全可达终点时为空。
         * @throws std::invalid_argument 地图不匹配、安全半径或容差非法。
         */
        std::vector<MapLocation> Plan(
            const Costmap2D & planning,
            const Costmap2D & source,
            double clearance_radius,
            const MapLocation & start,
            const MapLocation & goal,
            bool include_unknown_clearance = true,
            double goal_tolerance = 0.0) const;

        /** 连续几何为安全判据；先验证实际起点到起点格中心的连接。 */
        // terminal_geometry 只约束原目标和容差替代终点，可包含当前观测及停车容差。
        // 所属格中心不安全时，从通过实际位置连续扫掠的邻接中心接入；start_geometry 检查全部当前观测。
        std::vector<MapLocation> Plan(
            const Costmap2D & planning, const CollisionGeometry & geometry,
            const PathPoint & actual_start, const MapLocation & start,
            const MapLocation & goal, double goal_tolerance = 0.0,
            const CollisionGeometry * terminal_geometry = nullptr,
            const CollisionGeometry * start_geometry = nullptr) const;

    private:
        double cost_travel_multiplier_;
        /**
         * @brief 维护最小 f 值队列、累计代价及父索引，搜索后回溯最终路径。
         *
         * 步长为 1 或 √2，乘以 (1 + 权重 × 目标格代价 / 252)。
         * 非负软代价使八方向几何启发式不高估；目标原格优先于容差候选。
         *
         * @param planning 膨胀规划图，253 及以上禁行。
         * @param source 原图指针；nullptr 时跳过连续扫掠。
         * @param clearance_radius 扫掠半径，米。
         * @param start 起点下标。
         * @param goal 原目标下标。
         * @param include_unknown_clearance 未知格余量检查开关。
         * @param goal_tolerance 替代终点半径，米。
         * @return 起点至选中终点的路径，无路返回空。
         * @throws std::invalid_argument 容差非法。
         */
        std::vector<MapLocation> PlanImpl(
            const Costmap2D & planning,
            const Costmap2D * source,
            double clearance_radius,
            const MapLocation & start,
            const MapLocation & goal,
            bool include_unknown_clearance,
            double goal_tolerance, const CollisionGeometry * geometry = nullptr,
            const CollisionGeometry * terminal_geometry = nullptr,
            const PathPoint * actual_start = nullptr,
            const CollisionGeometry * start_geometry = nullptr) const;
        /**
         * @brief 把栅格下标映射到按行存储的一维索引。
         *
         * @param mx x 下标。
         * @param my y 下标。
         * @param size_x 每行格数。
         * @return my × size_x + mx；本函数不检查边界。
         */
        unsigned int GetIndex(unsigned int mx, unsigned int my, unsigned int size_x) const;

        /**
         * @brief 计算八邻域几何启发式，不计软代价。
         *
         * @param start 起点下标。
         * @param goal 目标下标。
         * @return max(dx,dy) + (√2 - 1) × min(dx,dy)，单位格。
         */
        double OctileDistance(const MapLocation & start, const MapLocation & goal) const;
    };

/* Local constants ---------------------------------------------------------*/
    constexpr unsigned char kInscribedInflatedObstacle = 253;
    constexpr unsigned char kLethalObstacle = 254;
}
