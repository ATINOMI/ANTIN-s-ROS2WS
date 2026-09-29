#pragma once

#include <vector>

#include "mini_nav_core/map/costmap_2d.hpp"
#include "mini_nav_core/map/inflation_layer.hpp"

namespace mini_nav_core
{
    /** 固定大小的滚动障碍图；坐标由调用者统一转换到 odom。 */
    class RollingObstacleGrid
    {
    public:
        RollingObstacleGrid(unsigned int width, unsigned int height, double resolution,
                            const InflationParameters & inflation);

        /** 将窗口移至机器人周围，重叠区域保留，新进入区域置为未知。 */
        void CenterOn(double x, double y);
        /** 清除全部观测；所有栅格重新成为未知。 */
        void Reset();
        /** 沿传感器射线清除空闲格，并记录扫描时刻。 */
        bool IntegrateRay(double sensor_x, double sensor_y,
                          double end_x, double end_y, double stamp);
        /** 在扫描端点标记障碍；调用方应先完成整帧射线清除。 */
        bool MarkObstacle(double x, double y, double stamp);
        /** 将超过 timeout 秒没有被观测的栅格置为未知。 */
        void Expire(double now, double timeout);
        /** 对当前观测障碍生成独立的膨胀图。 */
        Costmap2D Inflated() const;
        const Costmap2D & Raw() const;

    private:
        Costmap2D raw_;
        InflationParameters inflation_;
        std::vector<double> observed_at_;
        bool centered_ = false;
    };
}
