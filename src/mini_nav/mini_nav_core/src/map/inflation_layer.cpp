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
        constexpr unsigned char kInscribedCost = 253;
        constexpr unsigned char kLethalCost = 254;
        constexpr unsigned char kUnknownCost = 255;

        struct InflationOffset
        {
            std::int64_t dx;
            std::int64_t dy;
            unsigned char cost;
        };
    }

    Costmap2D InflateCostmap(
        const Costmap2D & source,
        const InflationParameters & parameters)
    {
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

        Costmap2D result = source;
        const auto width = static_cast<std::int64_t>(source.GetSizeInCellsX());
        const auto height = static_cast<std::int64_t>(source.GetSizeInCellsY());
        const double resolution = source.GetResolution();
        const auto max_dx = static_cast<std::int64_t>(std::min(
            static_cast<double>(width - 1),
            std::floor(parameters.inflation_radius / resolution)));
        const auto max_dy = static_cast<std::int64_t>(std::min(
            static_cast<double>(height - 1),
            std::floor(parameters.inflation_radius / resolution)));

        // 栅格偏移只计算一次；每个源障碍使用相同的欧氏距离和代价值。
        std::vector<InflationOffset> offsets;
        for (std::int64_t dy = -max_dy; dy <= max_dy; ++dy) {
            for (std::int64_t dx = -max_dx; dx <= max_dx; ++dx) {
                const double distance = std::hypot(
                    static_cast<double>(dx), static_cast<double>(dy)) * resolution;
                if (distance == 0.0 || distance > parameters.inflation_radius) {
                    continue;
                }
                unsigned char cost = kInscribedCost;
                if (distance > hard_radius) {
                    const double scaled = 252.0 * std::exp(
                        -parameters.cost_scaling_factor * (distance - hard_radius));
                    cost = static_cast<unsigned char>(std::max(1.0, std::floor(scaled)));
                }
                offsets.push_back({dx, dy, cost});
            }
        }

        for (std::int64_t y = 0; y < height; ++y) {
            for (std::int64_t x = 0; x < width; ++x) {
                if (source.GetCost(static_cast<unsigned int>(x), static_cast<unsigned int>(y)) !=
                    kLethalCost) {
                    continue;
                }
                for (const auto & offset : offsets) {
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
        return result;
    }
}
