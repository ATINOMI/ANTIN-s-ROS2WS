/**
 * @file costmap_display.hpp
 * @brief 内部栅格代价到 ROS/RViz 占据值的转换。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <cstdint>

namespace mini_nav_nodes
{
    /**
     * @brief 将内部代价转换为 RViz costmap 占据值。
     *
     * 99 对应车体硬安全区、100 对应障碍本体；转换只用于消息和显示，规划保留原代价。
     *
     * @param cost 内部 0..255 代价。
     * @return 255→-1、254→100、253→99、0→0、1..252→1..98。
     */
    inline constexpr std::int8_t CostToOccupancyValue(unsigned char cost)
    {
        if (cost == 255) {
            return -1;
        }
        if (cost == 254) {
            return 100;
        }
        if (cost == 253) {
            return 99;
        }
        if (cost == 0) {
            return 0;
        }
        return static_cast<std::int8_t>(1U + 97U * (cost - 1U) / 251U);
    }
}
