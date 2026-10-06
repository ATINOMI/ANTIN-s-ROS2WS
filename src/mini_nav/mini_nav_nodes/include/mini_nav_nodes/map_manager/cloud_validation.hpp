/**
 * @file cloud_validation.hpp
 * @brief 校验高度碰撞点云的 XYZ 数据布局，避免无效字节进入栅格。
 */
#pragma once

#include <algorithm>
#include <cstddef>
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace mini_nav_nodes
{
    inline bool ValidCollisionCloud(const sensor_msgs::msg::PointCloud2 & cloud)
    {
        if (cloud.is_bigendian || cloud.point_step < 12 ||
            static_cast<std::size_t>(cloud.width) * cloud.height > 100000 ||
            static_cast<std::size_t>(cloud.point_step) * cloud.width > cloud.row_step ||
            static_cast<std::size_t>(cloud.row_step) * cloud.height != cloud.data.size()) return false;
        for (const char * name : {"x", "y", "z"}) {
            const auto field = std::find_if(cloud.fields.begin(), cloud.fields.end(),
                [name](const auto & value) { return value.name == name; });
            if (field == cloud.fields.end() || field->datatype != sensor_msgs::msg::PointField::FLOAT32 ||
                field->count != 1 || field->offset + 4 > cloud.point_step) return false;
        }
        return true;
    }
}
