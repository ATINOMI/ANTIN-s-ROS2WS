/**
 * @file path_postprocessor.cpp
 * @brief 连续圆形扫掠、代价约束的路径简化和平滑。
 * @author Antinomy
 * @date 2026-10-01
 */

/* Includes -----------------------------------------------------------------------*/

#include "mini_nav_core/navigator/path_postprocessor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

/* Namespaces ---------------------------------------------------------------------*/

namespace mini_nav_core {
    namespace {

/* Constants ----------------------------------------------------------------------*/

        constexpr unsigned char kBlockedCost = 253;  // 规划器约定的硬禁行阈值；253 及以上视为不可通行。
        constexpr unsigned char kUnknownCost = 255;  // 规划器约定的未知代价阈值；255 视为未知或不可知。
        constexpr double kEpsilon = 1.0e-9;          // 计算误差容忍，避免浮点数比较时的边界问题。

/* Structures ---------------------------------------------------------------------*/

        /**
         * @brief 线段安全性及沿线软代价积分的内部结果。
         */
        struct SegmentResult {
            bool safe;    // 线段扫掠是否严格安全；不安全时 cost 为正无穷。
            double cost;  // 线段扫掠的积分软代价；不安全时为正无穷。
        };

/* Functions ----------------------------------------------------------------------*/

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

        // 二维 DDA 逐格积分；恰好穿过栅格角时，两侧格也必须可通行。
        /**
         * @brief 用 DDA 积分线段软代价，并检查中心线及圆形车体扫掠。
         *
         * 每格贡献为格内线长 × (1 + multiplier × cost / 252)。
         * 角点及沿格边行驶时同时检查相邻格，防止穿角或贴禁行格边界。
         *
         * @param source 原始几何碰撞地图。
         * @param planning 匹配几何的膨胀规划图，253 及以上禁行。
         * @param first 线段第一端点，世界坐标，米。
         * @param second 线段第二端点，世界坐标，米。
         * @param radius 车体安全半径，米。
         * @param multiplier 非负软代价权重。
         * @param include_unknown_clearance 是否检查圆盘与未知格的相交。
         * @return 安全标志和积分代价；不安全时 cost 为正无穷。
         */
        SegmentResult EvaluateSegment(const Costmap2D &source, const Costmap2D &planning,
                                      const PathPoint &first, const PathPoint &second,
                                      double radius, double multiplier,
                                      bool include_unknown_clearance,
                                      const CollisionGeometry * geometry = nullptr) {
            const SegmentResult invalid{false, std::numeric_limits<double>::infinity()};
            if (!std::isfinite(first.x) || !std::isfinite(first.y) || !std::isfinite(second.x) ||
                !std::isfinite(second.y)) {
                return invalid;
            }
            const double resolution = planning.GetResolution();
            const double x0 = (first.x - planning.GetOriginX()) / resolution;
            const double y0 = (first.y - planning.GetOriginY()) / resolution;
            const double x1 = (second.x - planning.GetOriginX()) / resolution;
            const double y1 = (second.y - planning.GetOriginY()) / resolution;
            const double dx = x1 - x0;
            const double dy = y1 - y0;
            const double length = std::hypot(second.x - first.x, second.y - first.y);
            int cell_x = static_cast<int>(std::floor(x0));
            int cell_y = static_cast<int>(std::floor(y0));
            const int step_x = (dx > 0.0) - (dx < 0.0);
            const int step_y = (dy > 0.0) - (dy < 0.0);
            const double infinity = std::numeric_limits<double>::infinity();
            double next_x = step_x == 0
                                ? infinity
                                : ((step_x > 0 ? cell_x + 1.0 - x0 : x0 - cell_x) / std::abs(dx));
            double next_y = step_y == 0
                                ? infinity
                                : ((step_y > 0 ? cell_y + 1.0 - y0 : y0 - cell_y) / std::abs(dy));
            const double delta_x = step_x == 0 ? infinity : 1.0 / std::abs(dx);
            const double delta_y = step_y == 0 ? infinity : 1.0 / std::abs(dy);
            const auto safe_cell = [&](int x, int y) {
                return x >= 0 && y >= 0 &&
                       planning.IsInBounds(static_cast<unsigned int>(x),
                                           static_cast<unsigned int>(y)) &&
                       (geometry ? source.GetCost(x, y) < kBlockedCost :
                                   planning.GetCost(x, y) < kBlockedCost);
            };
            const auto safe_point_cells = [&](double x, double y) {
                const int base_x = static_cast<int>(std::floor(x));
                const int base_y = static_cast<int>(std::floor(y));
                const bool on_x_edge = std::abs(x - std::round(x)) < kEpsilon;
                const bool on_y_edge = std::abs(y - std::round(y)) < kEpsilon;
                for (int side_y = 0; side_y <= static_cast<int>(on_y_edge); ++side_y) {
                    for (int side_x = 0; side_x <= static_cast<int>(on_x_edge); ++side_x) {
                        if (!safe_cell(base_x - side_x, base_y - side_y)) {
                            return false;
                        }
                    }
                }
                return true;
            };
            if (!safe_point_cells(x0, y0) || !safe_point_cells(x1, y1)) {
                return invalid;
            }
            double t = 0.0;
            double total = 0.0;
            while (true) {
                if (!safe_cell(cell_x, cell_y) ||
                    (step_x == 0 && std::abs(x0 - std::round(x0)) < kEpsilon &&
                     !safe_cell(cell_x - 1, cell_y)) ||
                    (step_y == 0 && std::abs(y0 - std::round(y0)) < kEpsilon &&
                     !safe_cell(cell_x, cell_y - 1))) {
                    return invalid;
                }
                const double next_t = std::min({1.0, next_x, next_y});
                const double cell_cost = planning.GetCost(cell_x, cell_y);
                total +=
                    std::max(0.0, next_t - t) * length * (1.0 + multiplier * cell_cost / 252.0);
                if (next_t >= 1.0) {
                    break;
                }
                if (std::abs(next_x - next_y) < kEpsilon) {
                    if (!safe_cell(cell_x + step_x, cell_y) ||
                        !safe_cell(cell_x, cell_y + step_y)) {
                        return invalid;
                    }
                    cell_x += step_x;
                    cell_y += step_y;
                    next_x += delta_x;
                    next_y += delta_y;
                } else if (next_x < next_y) {
                    cell_x += step_x;
                    next_x += delta_x;
                } else {
                    cell_y += step_y;
                    next_y += delta_y;
                }
                t = next_t;
            }
            return (geometry ? geometry->IsClear(first, second) :
                    SweepIsClear(source, first, second, radius, include_unknown_clearance))
                       ? SegmentResult{true, total} : invalid;
        }
    } // namespace

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

    bool CollisionGeometry::IsClear(const PathPoint & first, const PathPoint & second,
                                    double extra_radius, CollisionConflict * conflict) const {
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
                                                 bool include_unknown_clearance,
                                                 const CollisionGeometry * geometry) {
        if (!std::isfinite(clearance_radius) || clearance_radius <= 0.0 ||
            !std::isfinite(cost_travel_multiplier) || cost_travel_multiplier < 0.0 ||
            source.GetSizeInCellsX() != planning.GetSizeInCellsX() ||
            source.GetSizeInCellsY() != planning.GetSizeInCellsY() ||
            source.GetResolution() != planning.GetResolution() ||
            source.GetOriginX() != planning.GetOriginX() ||
            source.GetOriginY() != planning.GetOriginY()) {
            throw std::invalid_argument(
                "Path postprocessor needs matching maps and valid parameters");
        }
        if (raw_path.empty()) {
            return {};
        }

        std::vector<PathPoint> raw;
        raw.reserve(raw_path.size());
        for (const auto &cell : raw_path) {
            if (!source.IsInBounds(cell.x, cell.y)) {
                return {};
            }
            PathPoint point{};
            source.MapToWorld(cell.x, cell.y, point.x, point.y);
            raw.push_back(point);
        }
        const auto evaluate = [&](const PathPoint &first, const PathPoint &second) {
            auto result = EvaluateSegment(source, planning, first, second, clearance_radius,
                                   cost_travel_multiplier, include_unknown_clearance, geometry);
            return result;
        };

        // 原始路径也必须连续安全；栅格中心安全不等于车体沿两中心运动安全。
        std::vector<double> raw_prefix(raw.size(), 0.0);
        if (!evaluate(raw.front(), raw.front()).safe) {
            return {};
        }
        for (std::size_t i = 1; i < raw.size(); ++i) {
            const auto segment = evaluate(raw[i - 1], raw[i]);
            if (!segment.safe) {
                return {};
            }
            raw_prefix[i] = raw_prefix[i - 1] + segment.cost;
        }
        if (raw.size() < 3) {
            return raw;
        }

        // 递归分割的迭代形式：整段捷径不安全或软代价更高时保留偏离最大的原始点。
        std::vector<bool> keep(raw.size(), false);
        keep.front() = true;
        keep.back() = true;
        std::vector<std::pair<std::size_t, std::size_t>> pending{{0, raw.size() - 1}};
        while (!pending.empty()) {
            const auto [begin, end] = pending.back();
            pending.pop_back();
            if (end <= begin + 1) {
                continue;
            }
            const auto shortcut = evaluate(raw[begin], raw[end]);
            if (shortcut.safe && shortcut.cost <= raw_prefix[end] - raw_prefix[begin] + kEpsilon) {
                continue;
            }
            std::size_t split = begin + 1;
            double largest_deviation = -1.0;
            for (std::size_t i = begin + 1; i < end; ++i) {
                const double deviation = PointToSegmentSquared(raw[i], raw[begin], raw[end]);
                if (deviation > largest_deviation) {
                    largest_deviation = deviation;
                    split = i;
                }
            }
            keep[split] = true;
            pending.emplace_back(begin, split);
            pending.emplace_back(split, end);
        }
        std::vector<PathPoint> simplified;
        for (std::size_t i = 0; i < raw.size(); ++i) {
            if (keep[i]) {
                simplified.push_back(raw[i]);
            }
        }

        // 加密简化线段后逐点小幅移动，使 RViz 的最终线条真正形成圆滑转弯。
        std::vector<PathPoint> smooth;
        smooth.push_back(simplified.front());
        const double spacing = std::max(0.10, 2.0 * source.GetResolution());
        for (std::size_t i = 1; i < simplified.size(); ++i) {
            const auto &first = simplified[i - 1];
            const auto &second = simplified[i];
            const auto count = std::max<std::size_t>(
                1, static_cast<std::size_t>(
                       std::ceil(std::hypot(second.x - first.x, second.y - first.y) / spacing)));
            for (std::size_t step = 1; step <= count; ++step) {
                const double ratio = static_cast<double>(step) / count;
                smooth.push_back({first.x + ratio * (second.x - first.x),
                                  first.y + ratio * (second.y - first.y)});
            }
        }
        for (int pass = 0; pass < 5; ++pass) {
            bool changed = false;
            for (std::size_t i = 1; i + 1 < smooth.size(); ++i) {
                const PathPoint candidate{
                    0.25 * smooth[i - 1].x + 0.5 * smooth[i].x + 0.25 * smooth[i + 1].x,
                    0.25 * smooth[i - 1].y + 0.5 * smooth[i].y + 0.25 * smooth[i + 1].y};
                const auto old_first = evaluate(smooth[i - 1], smooth[i]);
                const auto old_second = evaluate(smooth[i], smooth[i + 1]);
                const auto new_first = evaluate(smooth[i - 1], candidate);
                const auto new_second = evaluate(candidate, smooth[i + 1]);
                if (old_first.safe && old_second.safe && new_first.safe && new_second.safe &&
                    new_first.cost + new_second.cost <=
                        old_first.cost + old_second.cost + kEpsilon) {
                    changed |=
                        std::hypot(candidate.x - smooth[i].x, candidate.y - smooth[i].y) > kEpsilon;
                    smooth[i] = candidate;
                }
            }
            if (!changed) {
                break;
            }
        }

        double final_cost = 0.0;
        for (std::size_t i = 1; i < smooth.size(); ++i) {
            const auto segment = evaluate(smooth[i - 1], smooth[i]);
            if (!segment.safe) {
                return raw;
            }
            final_cost += segment.cost;
        }
        return final_cost <= raw_prefix.back() + kEpsilon ? smooth : raw;
    }
} // namespace mini_nav_core
