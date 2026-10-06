#pragma once

namespace mini_nav_core::nav_types {
    /// 地图坐标系中的连续路径点，单位为米。
    /**
     * @brief 世界坐标系连续路径点，x、y 单位为米；参考帧由调用者统一。
     */
    struct Point2D {
        double x;
        double y;
    };

}

namespace mini_nav_core {
    using PathPoint = nav_types::Point2D;
}
