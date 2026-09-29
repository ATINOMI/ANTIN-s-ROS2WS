#pragma once

#include <vector>

#include "mini_nav_core/map/costmap_2d.hpp"

namespace mini_nav_core {
    /// 地图坐标系中的连续路径点，单位为米。
    struct PathPoint {
        double x;
        double y;
    };

    /** 检查圆形车体沿线段移动时是否碰到障碍、可选未知格或地图外侧。 */
    bool IsCircularSweepClear(const Costmap2D &source, const PathPoint &first,
                              const PathPoint &second, double clearance_radius,
                              bool include_unknown_clearance = true);

    /**
     * @brief 简化并平滑 A* 路径，同时检查每段的栅格、圆形车体扫掠和软代价。
     *
     * source 用于几何碰撞检查，planning 用于硬禁行区及积分软代价。
     * include_unknown_clearance 只控制车体与相邻未知格的余量；未知格仍禁行。
     * 捷径只有在安全且不增加积分代价时才被采用；平滑点同样逐个检查。
     * 最终检查失败时返回已验证的原始路径；原始路径不安全时返回空路径。
     */
    std::vector<PathPoint> SimplifyAndSmoothPath(const Costmap2D &source, const Costmap2D &planning,
                                                 const std::vector<MapLocation> &raw_path,
                                                 double clearance_radius,
                                                 double cost_travel_multiplier,
                                                 bool include_unknown_clearance = true);
} // namespace mini_nav_core
