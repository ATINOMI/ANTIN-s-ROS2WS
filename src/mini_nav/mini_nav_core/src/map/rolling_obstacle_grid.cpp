/**
 * @file rolling_obstacle_grid.cpp
 * @brief odom 系固定窗口滚动、射线观测与过期维护。
 * @author Antinomy
 * @date 2026-10-01
 */

/* Includes -----------------------------------------------------------------------*/

#include "mini_nav_core/map/rolling_obstacle_grid.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace mini_nav_core
{
    namespace
    {
        constexpr unsigned char kUnknown = 255;
        constexpr unsigned char kObstacle = 254;
        constexpr double kNeverObserved = -std::numeric_limits<double>::infinity();
    }

/* Functions ----------------------------------------------------------------------*/

    /**
     * @brief 建立全未知的固定尺寸滚动窗口，并立即校验膨胀参数。
     *
     * @param width x 方向格数，必须非零。
     * @param height y 方向格数，必须非零。
     * @param resolution 格边长，单位米，必须为有限正数。
     * @param inflation 车体半径、安全余量及代价传播参数。
     * @throws std::invalid_argument 地图几何或膨胀参数非法；超出容量时可能抛出 std::length_error。
     */
    RollingObstacleGrid::RollingObstacleGrid(
        unsigned int width, 
        unsigned int height, 
        double resolution,
        const InflationParameters & inflation): raw_(width, height, resolution, 0.0, 0.0, kUnknown), 
                                                inflation_(inflation),
                                                observed_at_(raw_.GetCellCount(), kNeverObserved),
                                                points_(raw_.GetCellCount()),
                                                opaque_cells_(raw_.GetCellCount(), false)
    {
        // 在首次使用前校验膨胀参数，避免扫描回调中才暴露配置错误。
        (void)InflateCostmap(raw_, inflation_);
    }

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
    void RollingObstacleGrid::CenterOn(double x, double y)
    {
        if (!std::isfinite(x) || !std::isfinite(y)) 
        {
            throw std::invalid_argument("Robot position must be finite");
        }

        const double resolution = raw_.GetResolution();

        // 计算窗口左下角的期望原点坐标，使机器人位于窗口中心。
        const double desired_x = x - raw_.GetSizeInCellsX() * resolution / 2.0;
        const double desired_y = y - raw_.GetSizeInCellsY() * resolution / 2.0;

        // 首次居中时直接设置原点，避免计算位移和滚动。
        if (!centered_) 
        {
            raw_ = Costmap2D(raw_.GetSizeInCellsX(), 
                             raw_.GetSizeInCellsY(), 
                             resolution,
                             desired_x, 
                             desired_y, 
                             kUnknown);

            std::fill(observed_at_.begin(), observed_at_.end(), kNeverObserved);
            for (auto & points : points_) points.clear();
            std::fill(opaque_cells_.begin(), opaque_cells_.end(), false);
            centered_ = true;
            return;
        }

        // 计算当前原点与期望原点的偏移量（以格为单位）。
        const double offset_x = (desired_x - raw_.GetOriginX()) / resolution;
        const double offset_y = (desired_y - raw_.GetOriginY()) / resolution;

        // 如果偏移量超过窗口尺寸，则直接重建窗口，丢弃全部观测。
        if (std::abs(offset_x) >= raw_.GetSizeInCellsX() ||
            std::abs(offset_y) >= raw_.GetSizeInCellsY()) 
        {
            raw_ = Costmap2D(raw_.GetSizeInCellsX(), 
                             raw_.GetSizeInCellsY(), 
                             resolution,
                             desired_x, 
                             desired_y, 
                             kUnknown);

            std::fill(observed_at_.begin(), observed_at_.end(), kNeverObserved);
            for (auto & points : points_) points.clear();
            std::fill(opaque_cells_.begin(), opaque_cells_.end(), false);
            return;
        }
        // 只在位移达到整格时滚动，避免定位抖动让窗口在半格附近来回跳。
        // 补偿整格边界上的浮点舍入误差，不改变未满一格时的截断规则。
        constexpr double kCellOffsetTolerance = 1.0e-9;

        // 计算整格位移量，向零舍入。
        const int shift_x = static_cast<int>(offset_x + (offset_x >= 0.0 ? kCellOffsetTolerance : -kCellOffsetTolerance));
        const int shift_y = static_cast<int>(offset_y + (offset_y >= 0.0 ? kCellOffsetTolerance : -kCellOffsetTolerance));

        // 如果没有整格位移，则不滚动窗口。
        if (shift_x == 0 && shift_y == 0) 
        {
            return;
        }
        // 创建一个新的窗口，并将原窗口中仍在新窗口范围内的栅格和观测时间搬移过去。
        Costmap2D moved(raw_.GetSizeInCellsX(), 
                        raw_.GetSizeInCellsY(), 
                        resolution,
                        raw_.GetOriginX() + shift_x * resolution,
                        raw_.GetOriginY() + shift_y * resolution, kUnknown);

        // 搬移观测时间，超出新窗口范围的格恢复为未知。
        std::vector<double> moved_observed_at(raw_.GetCellCount(), kNeverObserved);
        std::vector<std::vector<PathPoint>> moved_points(raw_.GetCellCount());
        std::vector<bool> moved_opaque(raw_.GetCellCount(), false);
        const std::size_t width = raw_.GetSizeInCellsX();

        for (unsigned int y_cell = 0; y_cell < raw_.GetSizeInCellsY(); ++y_cell) 
        {

            for (unsigned int x_cell = 0; x_cell < raw_.GetSizeInCellsX(); ++x_cell) 
            {

                // 计算原窗口栅格在新窗口中的位置，并且得到新窗口的栅格下标。
                const int new_x = static_cast<int>(x_cell) - shift_x;
                const int new_y = static_cast<int>(y_cell) - shift_y;

                if (new_x >= 0 && 
                    new_y >= 0 &&
                    new_x < static_cast<int>(raw_.GetSizeInCellsX()) &&
                    new_y < static_cast<int>(raw_.GetSizeInCellsY())) 
                    {

                    moved.SetCost(static_cast<unsigned int>(new_x),
                                  static_cast<unsigned int>(new_y), 
                                  raw_.GetCost(x_cell, y_cell));

                    moved_observed_at[static_cast<std::size_t>(new_y) * width + new_x] =
                        observed_at_[static_cast<std::size_t>(y_cell) * width + x_cell];
                    moved_opaque[static_cast<std::size_t>(new_y) * width + new_x] =
                        opaque_cells_[static_cast<std::size_t>(y_cell) * width + x_cell];
                }
            }
        }
        // 更新窗口和观测时间。
        raw_ = std::move(moved);
        for (std::size_t index = 0; index < points_.size(); ++index) {
            for (const auto & point : points_[index]) {
                unsigned int mx, my;
                if (raw_.WorldToMap(point.x, point.y, mx, my))
                    moved_points[static_cast<std::size_t>(my) * width + mx].push_back(point);
            }
        }
        points_ = std::move(moved_points);
        opaque_cells_ = std::move(moved_opaque);
        observed_at_ = std::move(moved_observed_at);
    }

    /**
     * @brief 将全部栅格与观测时间恢复为未知，保留窗口几何和居中状态。
     */
    void RollingObstacleGrid::Reset()
    {
        raw_.Fill(kUnknown);
        std::fill(observed_at_.begin(), observed_at_.end(), kNeverObserved);
        for (auto & points : points_) points.clear();
        std::fill(opaque_cells_.begin(), opaque_cells_.end(), false);
    }

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
    bool RollingObstacleGrid::IntegrateRay(
        double sensor_x, double sensor_y, double end_x, double end_y, double stamp)
    {

        unsigned int start_x = 0;
        unsigned int start_y = 0;

        // 检查窗口是否已居中，传感器和端点坐标及时间是否有限，并将传感器坐标转换为栅格下标。
        if (!centered_ || 
            !raw_.WorldToMap(sensor_x, sensor_y, start_x, start_y) ||
            !std::isfinite(end_x) || 
            !std::isfinite(end_y) || 
            !std::isfinite(stamp)) 
        {
            return false;
        }

        // 计算射线在 x 和 y 方向的增量，以及每个栅格的边长。
        const double dx = end_x - sensor_x;
        const double dy = end_y - sensor_y;
        const double resolution = raw_.GetResolution();
        int cell_x = static_cast<int>(start_x);
        int cell_y = static_cast<int>(start_y);

        // 计算射线在 x 和 y 方向的步长(正/负)，以及下一个栅格边界的坐标。
        const int step_x = (dx > 0.0) - (dx < 0.0);
        const int step_y = (dy > 0.0) - (dy < 0.0);
        const double infinity = std::numeric_limits<double>::infinity();

        // 寻找当前格子的下一个X和Y边界坐标，并计算到达这些边界所需的比例。
        const double x_boundary = raw_.GetOriginX() + (cell_x + (step_x > 0 ? 1 : 0)) * resolution;
        const double y_boundary = raw_.GetOriginY() + (cell_y + (step_y > 0 ? 1 : 0)) * resolution;

        // 计算到达下一个X和Y边界所需的比例，如果步长为0，则设置为无穷大。
        double next_x = step_x == 0 ? infinity : (x_boundary - sensor_x) / dx;
        double next_y = step_y == 0 ? infinity : (y_boundary - sensor_y) / dy;

        // 计算每个栅格的边长与射线增量的比值，用于更新比例。
        const double delta_x = step_x == 0 ? infinity : resolution / std::abs(dx);
        const double delta_y = step_y == 0 ? infinity : resolution / std::abs(dy);
        double fraction = 0.0;

        // 使用DDA算法遍历射线经过的栅格，直到射线离开窗口或达到终点。
        while (fraction <= 1.0 && 
               cell_x >= 0 && 
               cell_y >= 0 &&
               cell_x < static_cast<int>(raw_.GetSizeInCellsX()) &&
               cell_y < static_cast<int>(raw_.GetSizeInCellsY())) 
        {
            // 将当前栅格标记为0代价格。
            raw_.SetCost(static_cast<unsigned int>(cell_x),
                         static_cast<unsigned int>(cell_y), 0);
            // 记录观测时间，使用一维索引计算。
            observed_at_[static_cast<std::size_t>(cell_y) * raw_.GetSizeInCellsX() + cell_x] = stamp;
            points_[static_cast<std::size_t>(cell_y) * raw_.GetSizeInCellsX() + cell_x].clear();
            opaque_cells_[static_cast<std::size_t>(cell_y) * raw_.GetSizeInCellsX() + cell_x] = false;

            // 根据谁先碰到自己对应的边界，更新比例和下一个边界的比例。
            if (next_x < next_y) 
            {
                fraction = next_x;
                next_x += delta_x;
                cell_x += step_x;
            } 

            else if (next_y < next_x) 
            {
                fraction = next_y;
                next_y += delta_y;
                cell_y += step_y;
            } 

            else 
            {
                // 射线正好穿过栅格角点时只进入对角格，不清空旁边两格。
                fraction = next_x;
                next_x += delta_x;
                next_y += delta_y;
                cell_x += step_x;
                cell_y += step_y;
            }
        }
        return true;
    }

    /**
     * @brief 把图内扫描命中点写成 254 障碍并更新观测时间。
     *
     * @param x odom 系命中点 x，米。
     * @param y odom 系命中点 y，米。
     * @param stamp 观测时刻，秒。
     * @return 标记成功为 true；未居中、越界或时间非有限为 false。
     */
    bool RollingObstacleGrid::MarkObstacle(double x, double y, double stamp, bool precise)
    {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!centered_ || !std::isfinite(stamp) || !raw_.WorldToMap(x, y, mx, my)) 
        {
            return false;
        }
        const auto index = static_cast<std::size_t>(my) * raw_.GetSizeInCellsX() + mx;
        if (!precise) {
            points_[index].clear();
            opaque_cells_[index] = true;
        } else {
            if (observed_at_[index] != stamp) points_[index].clear();
            points_[index].push_back({x, y});
        }
        raw_.SetCost(mx, my, kObstacle);
        observed_at_[static_cast<std::size_t>(my) * raw_.GetSizeInCellsX() + mx] = stamp;
        return true;
    }

    /**
     * @brief 把观测超时或时间回跳后不再可信的格恢复为未知。
     *
     * 过期意味着未知而非空闲；必须有新射线才能重新证明该区域空闲。
     *
     * @param now 当前时刻，秒，与观测时间同源。
     * @param timeout 允许保留观测的时长，有限正数，单位秒。
     * @throws std::invalid_argument 时间非有限或 timeout 不为正数。
     */
    void RollingObstacleGrid::Expire(double now, double timeout)
    {
        // 检查时间参数是否合法。
        if (!std::isfinite(now) || 
            !std::isfinite(timeout) || 
            timeout <= 0.0) 
        {
            throw std::invalid_argument("Observation expiry requires finite time and positive timeout");
        }

        // 遍历所有栅格，检查观测时间是否超时或回跳，如果是，则将其恢复为未知。
        for (std::size_t index = 0; index < observed_at_.size(); ++index) 
        {
            if (observed_at_[index] != kNeverObserved &&
                (now < observed_at_[index] || now - observed_at_[index] > timeout)) 
            {
                mini_nav_core::MapLocation location{};
                raw_.IndexToMap(index, location);
                raw_.SetCost(location.x, location.y, kUnknown);
                observed_at_[index] = kNeverObserved;
                points_[index].clear();
                opaque_cells_[index] = false;
            }
        }
    }

    /**
     * @brief 从当前原始观测生成独立的车体膨胀安全图。
     * @return 与窗口几何一致的膨胀图；原始观测不变。
     */
    Costmap2D RollingObstacleGrid::Inflated() const
    {
        return InflateCostmap(raw_, inflation_);
    }

    /**
     * @brief 只读访问当前窗口的原始代价与几何。
     * @return 内部地图的常引用；随下一次窗口修改变化，不是快照。
     */
    const Costmap2D & RollingObstacleGrid::Raw() const
    {
        return raw_;
    }

    Costmap2D RollingObstacleGrid::CollisionGrid() const
    {
        auto coverage = raw_;
        for (std::size_t index = 0; index < points_.size(); ++index) {
            if (!opaque_cells_[index] && !points_[index].empty()) {
                MapLocation cell{};
                coverage.IndexToMap(index, cell);
                coverage.SetCost(cell.x, cell.y, 0);
            }
        }
        return coverage;
    }

    std::vector<PathPoint> RollingObstacleGrid::ObstaclePoints() const
    {
        std::vector<PathPoint> result;
        for (const auto & points : points_) result.insert(result.end(), points.begin(), points.end());
        return result;
    }
}
