/**
 * @file inflation_layer.cpp
 * @brief 按障碍格面积生成软代价，可选未知格硬膨胀并保留地图边界距离。
 *
 * 偏移代价只预计算一次，再叠加到各个禁行源周围；地图几何与
 * 原始占据信息由 Costmap2D 的副本保留，不依赖 ROS 消息或 Nav2 实现。
 * @author Antinomy
 * @date 2026-09-28
 */
#include "mini_nav_core/map/inflation_layer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace mini_nav_core
{
    namespace
    {
        /* 与规划器约定的三个特殊代价值：253 为硬安全区，254 为障碍源，
         * 255 为未知。改变其中任何一个，都必须同步检查 A* 的通行阈值。 */
        constexpr unsigned char kInscribedCost = 253;
        constexpr unsigned char kLethalCost = 254;
        constexpr unsigned char kUnknownCost = 255;

        /**
         * @brief 一个障碍源到目标栅格的相对偏移及对应膨胀代价。
         *
         * dx、dy 使用有符号栅格数，以表示障碍四周的偏移；
         * 同一组偏移可供地图中每个障碍复用。
         */
        struct InflationOffset
        {
            /// 相对障碍格的水平偏移，单位：格。
            std::int64_t dx;
            /// 相对障碍格的竖直偏移，单位：格。
            std::int64_t dy;
            /// 该距离对应的硬安全区或软膨胀代价。
            unsigned char cost;
        };
    }

    /**
     * @brief 复制原图并分别处理障碍软膨胀与未知边界硬安全区。
     * @param source 输入静态地图；254 是软膨胀源，255 可选传播硬安全区。
     * @param parameters 距离参数以米计，衰减系数以 1/米计。
     * @return 与 source 几何一致、原图未被修改的规划代价地图。
     * @throws std::invalid_argument 参数不是有限数，或半径、衰减范围无效时抛出。
     */
    Costmap2D InflateCostmap(
        const Costmap2D & source,
        const InflationParameters & parameters)
    {
        /*
         * 硬安全区需要同时覆盖机器人外接圆和附加余量。
         * 先验证全部参数，避免非法浮点数进入 floor、指数计算或栅格循环；
         * 膨胀半径小于硬安全半径时，也无法完整表达要求的禁行范围。
         */
        const double hard_radius = parameters.robot_radius + parameters.safety_margin;
        if (!std::isfinite(parameters.robot_radius) || parameters.robot_radius <= 0.0 ||
            !std::isfinite(parameters.safety_margin) || parameters.safety_margin < 0.0 ||
            !std::isfinite(hard_radius) ||
            !std::isfinite(parameters.inflation_radius) ||
            parameters.inflation_radius < hard_radius ||
            !std::isfinite(parameters.cost_scaling_factor) ||
            parameters.cost_scaling_factor <= 0.0) {
            throw std::invalid_argument("Invalid inflation radius, safety margin, or scaling factor");
        }

        /*
         * 从源地图复制，保留所有原始代价及几何信息。
         * 单轴偏移最多只需覆盖地图宽/高减一格：再远的目标必定越界。
         */
        Costmap2D result = source;
        const auto width = static_cast<std::int64_t>(source.GetSizeInCellsX());
        const auto height = static_cast<std::int64_t>(source.GetSizeInCellsY());
        const double resolution = source.GetResolution();
        const auto max_dx = static_cast<std::int64_t>(std::min(
            static_cast<double>(width - 1),
            std::ceil(parameters.inflation_radius / resolution + 0.5)));
        const auto max_dy = static_cast<std::int64_t>(std::min(
            static_cast<double>(height - 1),
            std::ceil(parameters.inflation_radius / resolution + 0.5)));

        const auto inflationCost = [&](double distance) {
            if (distance <= hard_radius) {
                return kInscribedCost;
            }
            const double scaled = 252.0 * std::exp(
                -parameters.cost_scaling_factor * (distance - hard_radius));
            return static_cast<unsigned char>(std::max(1.0, std::floor(scaled)));
        };

        /*
         * 栅格偏移只计算一次，每个禁行源复用同一张代价模板。
         * 距离取目标格中心到源格方形面积的最短距离，而不是格中心距；
         * 这样硬半径才覆盖车体圆盘与源格的相交情形。
         * 源格已在地图副本中保持 254 或 255，无需写入模板。
         * 硬安全区为 253；仅障碍源外圈按 252 * exp(-系数 * (距离 - 硬半径))
         * 衰减，并限制最小值为 1，避免影响半径内因取整而出现零代价。
         */
        std::vector<InflationOffset> offsets;
        for (std::int64_t dy = -max_dy; dy <= max_dy; ++dy) {
            for (std::int64_t dx = -max_dx; dx <= max_dx; ++dx) {
                const double distance = std::hypot(
                    std::max(0.0, std::abs(static_cast<double>(dx)) - 0.5),
                    std::max(0.0, std::abs(static_cast<double>(dy)) - 0.5)) * resolution;
                if (distance == 0.0 || distance > parameters.inflation_radius) {
                    continue;
                }
                offsets.push_back({dx, dy, inflationCost(distance)});
            }
        }

        /*
         * 只从源地图读取障碍和未知格，避免刚写出的代价再次传播。
         * 开启未知膨胀时只传播硬安全区，不制造软代价色带；多个源取最大代价。
         */
        for (std::int64_t y = 0; y < height; ++y) {
            for (std::int64_t x = 0; x < width; ++x) {
                const auto source_cost = source.GetCost(
                    static_cast<unsigned int>(x), static_cast<unsigned int>(y));
                if (source_cost != kLethalCost &&
                    !(parameters.inflate_around_unknown && source_cost == kUnknownCost)) {
                    continue;
                }
                for (const auto & offset : offsets) {
                    if (source_cost == kUnknownCost && offset.cost != kInscribedCost) {
                        continue;
                    }
                    const auto nx = x + offset.dx;
                    const auto ny = y + offset.dy;
                    if (nx < 0 || nx >= width || ny < 0 || ny >= height) {
                        continue;
                    }
                    const auto mx = static_cast<unsigned int>(nx);
                    const auto my = static_cast<unsigned int>(ny);
                    const auto old_cost = result.GetCost(mx, my);
                    if (old_cost != kUnknownCost && offset.cost > old_cost) {
                        result.SetCost(mx, my, offset.cost);
                    }
                }
            }
        }

        // 地图外侧没有占据信息：边缘到车体中心的距离按连续地图边界计算。
        // 已知格中心若距任一外边界不足硬半径，则禁止通行。
        for (std::int64_t y = 0; y < height; ++y) {
            for (std::int64_t x = 0; x < width; ++x) {
                const auto mx = static_cast<unsigned int>(x);
                const auto my = static_cast<unsigned int>(y);
                if (result.GetCost(mx, my) == kUnknownCost) {
                    continue;
                }
                const double edge_distance = std::min({
                    (static_cast<double>(x) + 0.5) * resolution,
                    (static_cast<double>(width - x) - 0.5) * resolution,
                    (static_cast<double>(y) + 0.5) * resolution,
                    (static_cast<double>(height - y) - 0.5) * resolution});
                if (edge_distance <= hard_radius) {
                    result.SetCost(mx, my, std::max(result.GetCost(mx, my), kInscribedCost));
                }
            }
        }
        return result;
    }
}
