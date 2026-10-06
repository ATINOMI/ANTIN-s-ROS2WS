#include "mini_nav_core/collision_checker/collision_checker.hpp"

#include <algorithm>
#include <cmath>

namespace mini_nav_core {
        /**
         * @brief 计算点到闭线段的最短距离平方，退化线段按单点处理。
         *
         * @param point 待查询世界坐标点，米。
         * @param first 线段第一端点，世界坐标，米。
         * @param second 线段第二端点，世界坐标，米。
         * @return 距离平方，m²。
         */
        double PointToSegmentSquared(const PathPoint &point, 
                                     const PathPoint &first,
                                     const PathPoint &second) 
        {
            // 计算x和y方向的差值
            const double dx = second.x - first.x;
            const double dy = second.y - first.y;
            const double length_squared = dx * dx + dy * dy;

            // 计算点在线段上的投影参数
            const double t =  length_squared == 0.0 ? 0.0
                                                    : std::clamp(( (point.x - first.x) * dx + (point.y - first.y) * dy ) /
                                                                        length_squared,
                                                                    0.0, 1.0);

            return std::pow(point.x - first.x - t * dx, 2) +
                   std::pow(point.y - first.y - t * dy, 2);
        }

    namespace {
        constexpr unsigned char kBlockedCost = 253;
        constexpr unsigned char kUnknownCost = 255;
        constexpr double kEpsilon = 1.0e-9;
        /**
         * @brief 通过坐标钳位计算点到轴对齐矩形的距离平方。
         *
         * @param point 查询点，米。
         * @param min_x 矩形左边界，米。
         * @param min_y 矩形下边界，米。
         * @param max_x 矩形右边界，米。
         * @param max_y 矩形上边界，米。
         * @return 距离平方，m²；点在矩形内或边界上时为零。
         */
        double PointToRectangleSquared(const PathPoint &point, double min_x, double min_y,
                                       double max_x, double max_y) {
            const double dx = point.x - std::clamp(point.x, min_x, max_x);
            const double dy = point.y - std::clamp(point.y, min_y, max_y);
            return dx * dx + dy * dy;
        }

        /**
         * @brief 按参数区间裁剪检查闭线段是否接触轴对齐矩形。
         *
         * @param first 线段第一端点，世界坐标，米。
         * @param second 线段第二端点，世界坐标，米。
         * @param min_x 矩形左边界，米。
         * @param min_y 矩形下边界，米。
         * @param max_x 矩形右边界，米。
         * @param max_y 矩形上边界，米。
         * @return 相交或接触为 true；平行且在矩形外为 false。
         */
        bool SegmentIntersectsRectangle(const PathPoint &first, const PathPoint &second,
                                        double min_x, double min_y, double max_x, double max_y) {
            double enter = 0.0;
            double leave = 1.0;
            const double dx = second.x - first.x;
            const double dy = second.y - first.y;
            const auto clip = [&](double p, double q) {
                if (p == 0.0) {
                    return q >= 0.0;
                }
                const double ratio = q / p;
                if (p < 0.0) {
                    if (ratio > leave)
                        return false;
                    enter = std::max(enter, ratio);
                } else {
                    if (ratio < enter)
                        return false;
                    leave = std::min(leave, ratio);
                }
                return true;
            };
            return clip(-dx, first.x - min_x) && clip(dx, max_x - first.x) &&
                   clip(-dy, first.y - min_y) && clip(dy, max_y - first.y) && enter <= leave;
        }

        // 线段到占据方格的最短距离：相交、端点到方格、四角到线段三种情形。
        /**
         * @brief 计算闭线段与占据格矩形面积之间的最短距离平方。
         *
         * @param first 线段第一端点，世界坐标，米。
         * @param second 线段第二端点，世界坐标，米。
         * @param min_x 矩形左边界，米。
         * @param min_y 矩形下边界，米。
         * @param max_x 矩形右边界，米。
         * @param max_y 矩形上边界，米。
         * @return 距离平方，m²；接触或相交为零。
         */
        double SegmentToRectangleSquared(const PathPoint &first, const PathPoint &second,
                                         double min_x, double min_y, double max_x, double max_y) {
            if (SegmentIntersectsRectangle(first, second, min_x, min_y, max_x, max_y)) {
                return 0.0;
            }
            double distance = std::min(PointToRectangleSquared(first, min_x, min_y, max_x, max_y),
                                       PointToRectangleSquared(second, min_x, min_y, max_x, max_y));
            for (const PathPoint corner : {PathPoint{min_x, min_y}, PathPoint{min_x, max_y},
                                           PathPoint{max_x, min_y}, PathPoint{max_x, max_y}}) {
                distance = std::min(distance, PointToSegmentSquared(corner, first, second));
            }
            return distance;
        }

        /**
         * @brief 检查圆形车体中心沿线段移动的扫掠是否与禁行格或图外相交。
         *
         * 遍历线段扩张包围盒中的格子，以线段到方格面积的距离判断，
         * 避免仅采样中心线漏掉格角碰撞；安全图已膨胀时应避免再次加入完整车体半径。
         *
         * @param source 几何碰撞地图；253 及以上视为禁行，未知可单独放宽。
         * @param first 线段第一端点，世界坐标，米。
         * @param second 线段第二端点，世界坐标，米。
         * @param radius 有限正车体半径，米，由外层接口验证。
         * @param include_unknown_clearance 是否把未知格面积也作为碰撞源。
         * @return 全部扫掠严格在图内且不接触所选禁行格时为 true。
         */
        bool SweepIsClear(const Costmap2D &source, const PathPoint &first, const PathPoint &second,
                          double radius, bool include_unknown_clearance, CollisionConflict * conflict = nullptr) {
            const double origin_x = source.GetOriginX();
            const double origin_y = source.GetOriginY();
            const double resolution = source.GetResolution();
            const double max_x = origin_x + source.GetSizeInCellsX() * resolution;
            const double max_y = origin_y + source.GetSizeInCellsY() * resolution;
            const auto inside = [&](const PathPoint &point) {
                return std::isfinite(point.x) && std::isfinite(point.y) &&
                       point.x > origin_x + radius && point.x < max_x - radius &&
                       point.y > origin_y + radius && point.y < max_y - radius;
            };
            // 收缩后的地图是凸集；两个端点在内即可保证整条中心线不越界。
            if (!inside(first) || !inside(second)) {
                if (conflict) {
                    const auto point = !inside(first) ? first : second;
                    *conflict = {"boundary", point, std::min({point.x-origin_x, max_x-point.x,
                        point.y-origin_y, max_y-point.y}), radius, false};
                }
                return false;
            }

            const auto min_cell_x = static_cast<int>(std::max(
                0.0, std::floor((std::min(first.x, second.x) - radius - origin_x) / resolution)));
            const auto max_cell_x = static_cast<int>(std::min(
                static_cast<double>(source.GetSizeInCellsX() - 1),
                std::floor((std::max(first.x, second.x) + radius - origin_x) / resolution)));
            const auto min_cell_y = static_cast<int>(std::max(
                0.0, std::floor((std::min(first.y, second.y) - radius - origin_y) / resolution)));
            const auto max_cell_y = static_cast<int>(std::min(
                static_cast<double>(source.GetSizeInCellsY() - 1),
                std::floor((std::max(first.y, second.y) + radius - origin_y) / resolution)));
            const double radius_squared = radius * radius;
            for (int y = min_cell_y; y <= max_cell_y; ++y) {
                for (int x = min_cell_x; x <= max_cell_x; ++x) {
                    const auto cost = source.GetCost(x, y);
                    // 关闭未知膨胀只放宽圆盘与邻近未知格的距离；规划图仍阻止中心线进入未知格。
                    if (cost < kBlockedCost ||
                        (!include_unknown_clearance && cost == kUnknownCost)) {
                        continue;
                    }
                    const double cell_min_x = origin_x + x * resolution;
                    const double cell_min_y = origin_y + y * resolution;
                    const double distance_squared = SegmentToRectangleSquared(
                        first, second, cell_min_x, cell_min_y, cell_min_x + resolution, cell_min_y + resolution);
                    if (distance_squared <= radius_squared + kEpsilon) {
                        if (conflict) *conflict = {cost == kUnknownCost ? "unknown_cell" : "occupied_cell",
                            {cell_min_x + resolution*.5, cell_min_y + resolution*.5},
                            std::sqrt(distance_squared), radius, false};
                        return false;
                    }
                }
            }
            return true;
        }

    }

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
                              bool include_unknown_clearance) {
        return std::isfinite(clearance_radius) && clearance_radius > 0.0 &&
               SweepIsClear(source, first, second, clearance_radius, include_unknown_clearance);
    }

    bool CollisionChecker::IsClear(const CollisionGeometry & geometry,
                                    const PathPoint & first, const PathPoint & second,
                                    double extra_radius, CollisionConflict * conflict) {
        const auto & grid = geometry.grid;
        const auto & points = geometry.points;
        const auto radius = geometry.radius;
        const auto observation_uncertainty = geometry.observation_uncertainty;
        const auto include_unknown = geometry.include_unknown;
        const auto point_radius = geometry.point_radius;
        const auto & points_from_grid_translation = geometry.points_from_grid_translation;
        const auto points_from_grid_yaw = geometry.points_from_grid_yaw;
        if (conflict) *conflict = {"invalid_geometry", first, 0.0, radius, false};
        if (!std::isfinite(radius) || radius <= 0.0 ||
            !std::isfinite(observation_uncertainty) || observation_uncertainty < 0.0 ||
            !std::isfinite(extra_radius) || extra_radius < 0.0 ||
            !SweepIsClear(grid, first, second, radius + extra_radius, include_unknown, conflict)) {
            return false;
        }
        if (!std::isfinite(point_radius) || point_radius < 0.0 ||
            !std::isfinite(points_from_grid_translation.x) ||
            !std::isfinite(points_from_grid_translation.y) ||
            !std::isfinite(points_from_grid_yaw)) return false;
        const double bound = (point_radius > 0.0 ? point_radius : radius) + extra_radius + observation_uncertainty;
        if (!std::isfinite(bound)) return false;
        const double c = std::cos(points_from_grid_yaw), s = std::sin(points_from_grid_yaw);
        const auto in_points_frame = [&](const PathPoint & point) {
            return PathPoint{points_from_grid_translation.x + c * point.x - s * point.y,
                points_from_grid_translation.y + s * point.x + c * point.y};
        };
        const auto points_first = in_points_frame(first), points_second = in_points_frame(second);
        for (const auto & point : points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                PointToSegmentSquared(point, points_first, points_second) <= bound * bound) {
                if (conflict) *conflict = {"dynamic_point", point,
                    std::sqrt(PointToSegmentSquared(point, points_first, points_second)), bound, false};
                return false;
            }
        }
        if (conflict) *conflict = {};
        return true;
    }

    bool CollisionGeometry::IsClear(const PathPoint & first, const PathPoint & second,
                                    double extra_radius, CollisionConflict * conflict) const {
        return CollisionChecker::IsClear(*this, first, second, extra_radius, conflict);
    }
}
