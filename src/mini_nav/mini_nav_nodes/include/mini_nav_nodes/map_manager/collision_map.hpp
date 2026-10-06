#pragma once

#include <cmath>
#include <memory>
#include <stdexcept>
#include "mini_nav_nodes/msg/collision_map.hpp"
#include "mini_nav_core/collision_checker/collision_geometry.hpp"

namespace mini_nav_nodes {
    /** ROS 快照转核心原始几何，拒绝软膨胀值以防输入类型混淆。 */
    inline std::unique_ptr<mini_nav_core::Costmap2D> DecodeCollisionMap(
        const msg::CollisionMap & message, const std::string & frame,
        std::vector<mini_nav_core::PathPoint> & points,
        const std::string & observation_frame = "") {
        const auto & grid = message.grid;
        const auto & q = grid.info.origin.orientation;
        if (!message.valid || grid.header.frame_id != frame ||
            (!message.points_frame_id.empty() && message.points_frame_id != frame &&
             message.points_frame_id != observation_frame) ||
            !std::isfinite(message.clearance_radius) || message.clearance_radius <= 0.0 ||
            !std::isfinite(grid.info.resolution) || grid.info.resolution <= 0.0 ||
            grid.info.width == 0 || grid.info.height == 0 ||
            grid.data.size() != static_cast<std::size_t>(grid.info.width) * grid.info.height ||
            grid.data.size() > 20000000 || message.points.size() > 200000 ||
            !std::isfinite(message.observation_uncertainty) || message.observation_uncertainty < 0.0 ||
            !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w) ||
            std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z) > 1e-6 ||
            std::abs(std::abs(q.w) - 1.0) > 1e-6) {
            throw std::invalid_argument("Invalid collision snapshot metadata");
        }
        auto result = std::make_unique<mini_nav_core::Costmap2D>(grid.info.width, grid.info.height,
            grid.info.resolution, grid.info.origin.position.x, grid.info.origin.position.y, 255);
        for (std::size_t i = 0; i < grid.data.size(); ++i) {
            const int value = grid.data[i];
            if (value != -1 && value != 0 && value != 100)
                throw std::invalid_argument("Collision snapshot must contain raw cells");
            mini_nav_core::MapLocation cell{};
            result->IndexToMap(i, cell);
            result->SetCost(cell.x, cell.y, value == -1 ? 255 : value == 100 ? 254 : 0);
        }
        std::vector<mini_nav_core::PathPoint> decoded;
        decoded.reserve(message.points.size());
        for (const auto & point : message.points) {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
                throw std::invalid_argument("Invalid collision endpoint");
            decoded.push_back({point.x, point.y});
        }
        points = std::move(decoded);
        return result;
    }
}
