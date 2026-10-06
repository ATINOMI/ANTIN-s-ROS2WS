/**
 * @file path_postprocessor.hpp
 * @brief 连续圆形扫掠、代价约束的路径简化和平滑。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <vector>

#include "mini_nav_core/map/costmap_2d.hpp"

namespace mini_nav_core {
    /// 地图坐标系中的连续路径点，单位为米。
    /**
     * @brief 世界坐标系连续路径点，x、y 单位为米；参考帧由调用者统一。
     */
    struct PathPoint {
        double x;
        double y;
    };

    /** 首个被检查到的冲突；distance 为中心线到对象的距离，required 为要求间距。 */
    struct CollisionConflict {
        const char * kind{"none"};
        PathPoint object{0.0, 0.0};
        double distance{0.0};
        double required{0.0};
        bool local{false};
    };

    /** 原始静态格/观测覆盖与连续动态端点；余量独立于显示膨胀参数。 */
    struct CollisionGeometry {
        const Costmap2D & grid;
        const std::vector<PathPoint> & points;
        double radius;
        double observation_uncertainty{0.0};
        bool include_unknown{true};
        // 端点可保留在独立观测帧；静态格和端点分别使用各自的车体误差预算。
        double point_radius{0.0};
        PathPoint points_from_grid_translation{0.0, 0.0};
        double points_from_grid_yaw{0.0};

        /** 验证整条闭线段的圆盘扫掠，extra_radius 用于圆弧到弦的误差补偿。 */
        bool IsClear(const PathPoint & first, const PathPoint & second,
                     double extra_radius = 0.0, CollisionConflict * conflict = nullptr) const;
    };

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

    /**
     * @brief 简化并平滑栅格路径，逐段检查硬禁行、扫掠和积分软代价。
     *
     * 先验证原始路径，再通过迭代分割保留必要折点；加密后最多做五轮局部平滑。
     * 每次改点都验证相邻两段，最终仍复查整条路径，不能以视觉平滑代替安全验证。
     *
     * @param source 原始碰撞地图。
     * @param planning 与 source 尺寸、分辨率及原点完全一致的膨胀规划图。
     * @param raw_path A* 栅格中心路径；空输入直接返回空。
     * @param clearance_radius 有限正安全半径，米。
     * @param cost_travel_multiplier 有限非负软代价权重。
     * @param include_unknown_clearance 是否检查车体与邻近未知格的余量；规划中心线仍不得进入未知格。
     * @return 安全且不增加代价的连续路径；最终验证失败回退原始中心路径，原始路径不安全返回空。
     * @throws std::invalid_argument 地图几何不匹配或半径、代价权重非法。
     */
    std::vector<PathPoint> SimplifyAndSmoothPath(const Costmap2D &source, const Costmap2D &planning,
                                                 const std::vector<MapLocation> &raw_path,
                                                 double clearance_radius,
                                                 double cost_travel_multiplier,
                                                 bool include_unknown_clearance = true,
                                                 const CollisionGeometry * geometry = nullptr);
} // namespace mini_nav_core
