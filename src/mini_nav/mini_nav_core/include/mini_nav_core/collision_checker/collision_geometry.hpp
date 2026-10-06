#pragma once

#include <vector>
#include "mini_nav_core/map/costmap_2d.hpp"
#include "mini_nav_core/nav_types/point_2d.hpp"

namespace mini_nav_core {
    /** 首个被检查到的冲突；distance 为中心线到对象的距离，required 为要求间距。 */
    struct CollisionConflict {
        const char * kind{"none"};
        PathPoint object{0.0, 0.0};
        double distance{0.0};
        double required{0.0};
        bool local{false};
    };

    /** 原始静态格/观测覆盖与连续动态端点；余量独立于显示膨胀参数。 */
    struct CollisionGeometry {
        const Costmap2D & grid;
        const std::vector<PathPoint> & points;
        double radius;
        double observation_uncertainty{0.0};
        bool include_unknown{true};
        // 端点可保留在独立观测帧；静态格和端点分别使用各自的车体误差预算。
        double point_radius{0.0};
        PathPoint points_from_grid_translation{0.0, 0.0};
        double points_from_grid_yaw{0.0};

        /** 验证整条闭线段的圆盘扫掠，extra_radius 用于圆弧到弦的误差补偿。 */
        bool IsClear(const PathPoint & first, const PathPoint & second,
                     double extra_radius = 0.0, CollisionConflict * conflict = nullptr) const;
    };

}
