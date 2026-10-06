/**
 * @file inflation_layer.cpp
 * @brief 按 Nav2 1.3.12 的格中心距离、距离分组传播和未知格规则生成膨胀图。
 *
 * 适配官方 InflationLayer 的全图更新算法，保留上游 BSD 许可；
 * 核心仍只操作 Costmap2D，不依赖 ROS 消息或官方运行库。
 * @author Antinomy
 * @date 2026-09-28
 */

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/

/* Includes -----------------------------------------------------------------------*/

#include "mini_nav_core/map/inflation_layer.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <stdexcept>
#include <vector>

namespace mini_nav_core
{
    namespace
    {

/* Constants ----------------------------------------------------------------------*/

        /* 与规划器约定的三个特殊代价值：253 为硬安全区，254 为障碍源，
         * 255 为未知。改变其中任何一个，都必须同步检查 A* 的通行阈值。 */
        constexpr unsigned char kInscribedCost = 253;
        constexpr unsigned char kLethalCost = 254;
        constexpr unsigned char kUnknownCost = 255;

/* Structures ---------------------------------------------------------------------*/

        /**
         * @brief 待膨胀格及它首次传播来源的栅格坐标。
         */
        struct InflationCell
        {
            unsigned int x;
            unsigned int y;
            unsigned int source_x;
            unsigned int source_y;
        };
    }

/* Functions ----------------------------------------------------------------------*/

    /**
     * @brief 复制原图，按官方格中心距离顺序传播确定障碍和可选未知源。
     * @param source 输入地图；254 是障碍，255 可选成为完整膨胀源。
     * @param parameters 距离参数以米计，衰减系数以 1/米计。
     * @return 与 source 几何一致的独立规划图；不修改输入。
     * @throws std::invalid_argument 半径或衰减参数非法时抛出。
     */
    Costmap2D InflateCostmap(
        const Costmap2D & source,
        const InflationParameters & parameters)
    {
        // 膨胀内切半径与真实车体外接圆分开；零值保留原有安全图行为。
        const double body_radius = parameters.robot_radius + parameters.safety_margin;
        const double hard_radius = parameters.inscribed_radius > 0.0 ?
            parameters.inscribed_radius : body_radius;
        if (!std::isfinite(parameters.robot_radius) ||
            parameters.robot_radius <= 0.0 ||
            !std::isfinite(parameters.safety_margin) ||
            parameters.safety_margin < 0.0 ||
            !std::isfinite(body_radius) ||
            !std::isfinite(parameters.inscribed_radius) ||
            parameters.inscribed_radius < 0.0 ||
            !std::isfinite(parameters.inflation_radius) ||
            parameters.inflation_radius < body_radius ||
            parameters.inflation_radius < hard_radius ||
            !std::isfinite(parameters.cost_scaling_factor) ||
            parameters.cost_scaling_factor <= 0.0) {
            throw std::invalid_argument("Invalid inflation radius, safety margin, or scaling factor");
        }

        Costmap2D result = source;
        const auto width = source.GetSizeInCellsX();
        const auto height = source.GetSizeInCellsY();
        const double resolution = source.GetResolution();
        // 与官方 Costmap2D::cellDistance 一致：外半径向上取整到整格。
        const double cell_radius = std::ceil(parameters.inflation_radius / resolution);
        std::vector<bool> seen(source.GetCellCount(), false);
        // 同距离格保留入队顺序；首次访问锁定来源，不能用所有源逐格取最大值替代。
        std::map<double, std::vector<InflationCell>> pending;

        for (unsigned int y = 0; y < height; ++y) {
            for (unsigned int x = 0; x < width; ++x) {
                const auto cost = source.GetCost(x, y);
                if (cost == kLethalCost ||
                    (parameters.inflate_around_unknown && cost == kUnknownCost)) {
                    pending[0.0].push_back({x, y, x, y});
                }
            }
        }

        const auto enqueue = [&](unsigned int x, unsigned int y,
                                 unsigned int source_x, unsigned int source_y) {
            const auto index = static_cast<std::size_t>(y) * width + x;
            if (seen[index]) {
                return;
            }
            const double dx = std::abs(static_cast<double>(x) - source_x);
            const double dy = std::abs(static_cast<double>(y) - source_y);
            const double distance = std::hypot(dx, dy);
            if (distance <= cell_radius) {
                pending[distance].push_back({x, y, source_x, source_y});
            }
        };

        // 距离组顺序与官方整数距离分组相同；无需为整个半径建立二维缓存。
        for (auto & bin : pending) {
            const double distance = bin.first;
            unsigned char cost;
            if (distance == 0.0) {
                cost = kLethalCost;
            } else if (distance * resolution <= hard_radius) {
                cost = kInscribedCost;
            } else {
                // 与官方 computeCost 一致，直接截断；极低软代价允许为零。
                const double factor = std::exp(-1.0 * parameters.cost_scaling_factor *
                                              (distance * resolution - hard_radius));
                cost = static_cast<unsigned char>((kInscribedCost - 1) * factor);
            }
            for (std::size_t i = 0; i < bin.second.size(); ++i) {
                // 后续入队可能扩容当前组，先复制坐标，避免引用失效。
                const auto cell = bin.second[i];
                const auto index = static_cast<std::size_t>(cell.y) * width + cell.x;
                if (seen[index]) {
                    continue;
                }
                seen[index] = true;
                const auto old_cost = result.GetCost(cell.x, cell.y);
                // 对齐官方 inflate_unknown=false：未知格可接收硬代价，拒绝软代价。
                if (old_cost == kUnknownCost && cost >= kInscribedCost) {
                    result.SetCost(cell.x, cell.y, cost);
                } else {
                    result.SetCost(cell.x, cell.y, std::max(old_cost, cost));
                }

                // 官方传播顺序为左、下、右、上，同距离竞争时顺序影响首次来源。
                if (cell.x > 0) {
                    enqueue(cell.x - 1, cell.y, cell.source_x, cell.source_y);
                }
                if (cell.y > 0) {
                    enqueue(cell.x, cell.y - 1, cell.source_x, cell.source_y);
                }
                if (cell.x < width - 1) {
                    enqueue(cell.x + 1, cell.y, cell.source_x, cell.source_y);
                }
                if (cell.y < height - 1) {
                    enqueue(cell.x, cell.y + 1, cell.source_x, cell.source_y);
                }
            }
            std::vector<InflationCell>().swap(bin.second);
        }
        // 官方膨胀不把图外虚构成障碍；车体边界余量由原图连续扫掠另行检查。
        return result;
    }
}
