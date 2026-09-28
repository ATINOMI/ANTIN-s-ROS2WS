#pragma once

#include "mini_nav_core/map/costmap_2d.hpp"

namespace mini_nav_core
{
    /// 以机器人外接圆和额外安全距离构造静态规划代价地图。
    struct InflationParameters
    {
        double robot_radius = 0.24;
        double safety_margin = 0.05;
        double inflation_radius = 0.55;
        double cost_scaling_factor = 5.0;
    };

    /**
     * @brief 从原始静态图生成规划图，保留原图、地图几何和未知栅格。
     *
     * 障碍物格为 254；距障碍物中心不超过 robot_radius + safety_margin
     * 的已知格为 253；其余膨胀范围内的格为随欧氏距离衰减的 1..252。
     * 未知格保留为 255，且不作为膨胀源。
     * @throws std::invalid_argument 参数不合法时抛出。
     */
    Costmap2D InflateCostmap(
        const Costmap2D & source,
        const InflationParameters & parameters);
}
