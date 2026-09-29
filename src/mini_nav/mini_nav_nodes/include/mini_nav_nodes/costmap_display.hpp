#pragma once

#include <cstdint>

namespace mini_nav_nodes
{
    /**
     * @brief 将内部代价转换为 RViz costmap 配色使用的 OccupancyGrid 值。
     *
     * 99 表示车体不可进入的膨胀区，100 表示障碍本体；两者在 RViz 中
     * 分别显示为青色和紫色。规划器继续使用未经转换的 0 到 255 代价。
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
