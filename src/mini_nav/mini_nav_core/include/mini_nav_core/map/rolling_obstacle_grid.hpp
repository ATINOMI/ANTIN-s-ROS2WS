/**
 * @file rolling_obstacle_grid.hpp
 * @brief odom 系固定窗口滚动、射线观测与过期维护。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

#include <vector>

#include "mini_nav_core/map/costmap_2d.hpp"
#include "mini_nav_core/map/inflation_layer.hpp"
#include "mini_nav_core/nav_types/path.hpp"

namespace mini_nav_core
{
    /** 固定大小的滚动障碍图；坐标由调用者统一转换到 odom。 */
    class RollingObstacleGrid
    {
    public:
        /**
         * @brief 建立全未知的固定尺寸滚动窗口，并立即校验膨胀参数。
         *
         * @param width x 方向格数，必须非零。
         * @param height y 方向格数，必须非零。
         * @param resolution 格边长，单位米，必须为有限正数。
         * @param inflation 车体半径、安全余量及代价传播参数。
         * @throws std::invalid_argument 地图几何或膨胀参数非法；超出容量时可能抛出 std::length_error。
         */
        RollingObstacleGrid(unsigned int width, unsigned int height, double resolution,
                            const InflationParameters & inflation);

        /**
         * @brief 把窗口移动到机器人周围，按整格位移保留重叠观测。
         *
         * 首次设置原点；后续累计移动满一格才滚动。移出整幅窗口时清空全部观测。
         * 代价与观测时间必须同步搬移，否则旧障碍会出现在错误位置或错误过期。
         *
         * @param x odom 系机器人 x，单位米。
         * @param y odom 系机器人 y，单位米。
         * @throws std::invalid_argument 机器人位置非有限。
         */
        void CenterOn(double x, double y);
        /**
         * @brief 将全部栅格与观测时间恢复为未知，保留窗口几何和居中状态。
         */
        void Reset();
        /**
         * @brief 用二维 DDA 清除传感器到端点之间的可见栅格，并记录观测时刻。
         *
         * 射线离开窗口即停止；穿过角点只清除对角格。
         * 本函数会清除端点格；整帧射线完成后再统一 MarkObstacle，避免擦除其他束命中点。
         *
         * @param sensor_x odom 系传感器 x，米。
         * @param sensor_y odom 系传感器 y，米。
         * @param end_x odom 系射线端点 x，米；允许在窗口外。
         * @param end_y odom 系射线端点 y，米。
         * @param stamp 观测时刻，秒；应与 Expire 使用同一时基。
         * @return 窗口已居中且传感器在图内、端点及时间有限时为 true，否则不写图并返回 false。
         */
        bool IntegrateRay(double sensor_x, double sensor_y,
                          double end_x, double end_y, double stamp);
        /**
         * @brief 把图内扫描命中点写成 254 障碍并更新观测时间。
         *
         * @param x odom 系命中点 x，米。
         * @param y odom 系命中点 y，米。
         * @param stamp 观测时刻，秒。
         * @return 标记成功为 true；未居中、越界或时间非有限为 false。
         */
        bool MarkObstacle(double x, double y, double stamp, bool precise = true);
        /**
         * @brief 把观测超时或时间回跳后不再可信的格恢复为未知。
         *
         * 过期意味着未知而非空闲；必须有新射线才能重新证明该区域空闲。
         *
         * @param now 当前时刻，秒，与观测时间同源。
         * @param timeout 允许保留观测的时长，有限正数，单位秒。
         * @throws std::invalid_argument 时间非有限或 timeout 不为正数。
         */
        void Expire(double now, double timeout);
        /**
         * @brief 从当前原始观测生成独立的车体膨胀安全图。
         * @return 与窗口几何一致的膨胀图；原始观测不变。
         */
        Costmap2D Inflated() const;
        /**
         * @brief 只读访问当前窗口的原始代价与几何。
         * @return 内部地图的常引用；随下一次窗口修改变化，不是快照。
         */
        const Costmap2D & Raw() const;

        /** 覆盖图保留未知区，动态 lethal 改由同快照的连续端点表示。 */
        Costmap2D CollisionGrid() const;
        std::vector<PathPoint> ObstaclePoints() const;

    private:
        Costmap2D raw_;
        InflationParameters inflation_;
        std::vector<double> observed_at_;
        std::vector<std::vector<PathPoint>> points_;
        std::vector<bool> opaque_cells_;
        bool centered_ = false;
    };
}
