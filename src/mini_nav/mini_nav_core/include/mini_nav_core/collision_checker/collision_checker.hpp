#pragma once

#include "mini_nav_core/collision_checker/collision_geometry.hpp"

namespace mini_nav_core {
    /**
     * @brief 检查圆形车体沿闭线段的连续扫掠。
     *
     * @param source 碰撞用原图。
     * @param first 线段第一端点，世界坐标，米。
     * @param second 线段第二端点，世界坐标，米。
     * @param clearance_radius 有限正安全半径，米。
     * @param include_unknown_clearance 是否将未知格作为碰撞源。
     * @return 参数有效且扫掠严格安全为 true，否则 false。
     */
    bool IsCircularSweepClear(const Costmap2D &source, const PathPoint &first,
                              const PathPoint &second, double clearance_radius,
                              bool include_unknown_clearance = true);

    /** 规划和控制共用的连续车体碰撞检查入口。 */
    class CollisionChecker {
    public:
        static bool IsClear(const CollisionGeometry & geometry, const PathPoint & first,
                            const PathPoint & second, double extra_radius = 0.0,
                            CollisionConflict * conflict = nullptr);
    };

    double PointToSegmentSquared(const PathPoint & point, const PathPoint & first,
                                 const PathPoint & second);
}
