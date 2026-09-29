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

    RollingObstacleGrid::RollingObstacleGrid(
        unsigned int width, unsigned int height, double resolution,
        const InflationParameters & inflation)
      : raw_(width, height, resolution, 0.0, 0.0, kUnknown), inflation_(inflation),
        observed_at_(raw_.GetCellCount(), kNeverObserved)
    {
        // 在首次使用前校验膨胀参数，避免扫描回调中才暴露配置错误。
        (void)InflateCostmap(raw_, inflation_);
    }

    void RollingObstacleGrid::CenterOn(double x, double y)
    {
        if (!std::isfinite(x) || !std::isfinite(y)) {
            throw std::invalid_argument("Robot position must be finite");
        }
        const double resolution = raw_.GetResolution();
        const double desired_x = x - raw_.GetSizeInCellsX() * resolution / 2.0;
        const double desired_y = y - raw_.GetSizeInCellsY() * resolution / 2.0;
        if (!centered_) {
            raw_ = Costmap2D(raw_.GetSizeInCellsX(), raw_.GetSizeInCellsY(), resolution,
                             desired_x, desired_y, kUnknown);
            std::fill(observed_at_.begin(), observed_at_.end(), kNeverObserved);
            centered_ = true;
            return;
        }

        const double offset_x = (desired_x - raw_.GetOriginX()) / resolution;
        const double offset_y = (desired_y - raw_.GetOriginY()) / resolution;
        if (std::abs(offset_x) >= raw_.GetSizeInCellsX() ||
            std::abs(offset_y) >= raw_.GetSizeInCellsY()) {
            raw_ = Costmap2D(raw_.GetSizeInCellsX(), raw_.GetSizeInCellsY(), resolution,
                             desired_x, desired_y, kUnknown);
            std::fill(observed_at_.begin(), observed_at_.end(), kNeverObserved);
            return;
        }
        const int shift_x = static_cast<int>(std::round(offset_x));
        const int shift_y = static_cast<int>(std::round(offset_y));
        if (shift_x == 0 && shift_y == 0) {
            return;
        }
        Costmap2D moved(raw_.GetSizeInCellsX(), raw_.GetSizeInCellsY(), resolution,
                        raw_.GetOriginX() + shift_x * resolution,
                        raw_.GetOriginY() + shift_y * resolution, kUnknown);
        std::vector<double> moved_observed_at(raw_.GetCellCount(), kNeverObserved);
        const std::size_t width = raw_.GetSizeInCellsX();
        for (unsigned int y_cell = 0; y_cell < raw_.GetSizeInCellsY(); ++y_cell) {
            for (unsigned int x_cell = 0; x_cell < raw_.GetSizeInCellsX(); ++x_cell) {
                const int new_x = static_cast<int>(x_cell) - shift_x;
                const int new_y = static_cast<int>(y_cell) - shift_y;
                if (new_x >= 0 && new_y >= 0 &&
                    new_x < static_cast<int>(raw_.GetSizeInCellsX()) &&
                    new_y < static_cast<int>(raw_.GetSizeInCellsY())) {
                    moved.SetCost(static_cast<unsigned int>(new_x),
                                  static_cast<unsigned int>(new_y), raw_.GetCost(x_cell, y_cell));
                    moved_observed_at[static_cast<std::size_t>(new_y) * width + new_x] =
                        observed_at_[static_cast<std::size_t>(y_cell) * width + x_cell];
                }
            }
        }
        raw_ = std::move(moved);
        observed_at_ = std::move(moved_observed_at);
    }

    void RollingObstacleGrid::Reset()
    {
        raw_.Fill(kUnknown);
        std::fill(observed_at_.begin(), observed_at_.end(), kNeverObserved);
    }

    bool RollingObstacleGrid::IntegrateRay(
        double sensor_x, double sensor_y, double end_x, double end_y, double stamp)
    {
        unsigned int start_x = 0;
        unsigned int start_y = 0;
        if (!centered_ || !raw_.WorldToMap(sensor_x, sensor_y, start_x, start_y) ||
            !std::isfinite(end_x) || !std::isfinite(end_y) || !std::isfinite(stamp)) {
            return false;
        }
        const double dx = end_x - sensor_x;
        const double dy = end_y - sensor_y;
        const double resolution = raw_.GetResolution();
        int cell_x = static_cast<int>(start_x);
        int cell_y = static_cast<int>(start_y);
        const int step_x = (dx > 0.0) - (dx < 0.0);
        const int step_y = (dy > 0.0) - (dy < 0.0);
        const double infinity = std::numeric_limits<double>::infinity();
        const double x_boundary = raw_.GetOriginX() +
            (cell_x + (step_x > 0 ? 1 : 0)) * resolution;
        const double y_boundary = raw_.GetOriginY() +
            (cell_y + (step_y > 0 ? 1 : 0)) * resolution;
        double next_x = step_x == 0 ? infinity : (x_boundary - sensor_x) / dx;
        double next_y = step_y == 0 ? infinity : (y_boundary - sensor_y) / dy;
        const double delta_x = step_x == 0 ? infinity : resolution / std::abs(dx);
        const double delta_y = step_y == 0 ? infinity : resolution / std::abs(dy);
        double fraction = 0.0;
        while (fraction <= 1.0 && cell_x >= 0 && cell_y >= 0 &&
               cell_x < static_cast<int>(raw_.GetSizeInCellsX()) &&
               cell_y < static_cast<int>(raw_.GetSizeInCellsY())) {
            raw_.SetCost(static_cast<unsigned int>(cell_x),
                         static_cast<unsigned int>(cell_y), 0);
            observed_at_[static_cast<std::size_t>(cell_y) * raw_.GetSizeInCellsX() + cell_x] = stamp;
            if (next_x < next_y) {
                fraction = next_x;
                next_x += delta_x;
                cell_x += step_x;
            } else if (next_y < next_x) {
                fraction = next_y;
                next_y += delta_y;
                cell_y += step_y;
            } else {
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

    bool RollingObstacleGrid::MarkObstacle(double x, double y, double stamp)
    {
        unsigned int mx = 0;
        unsigned int my = 0;
        if (!centered_ || !std::isfinite(stamp) || !raw_.WorldToMap(x, y, mx, my)) {
            return false;
        }
        raw_.SetCost(mx, my, kObstacle);
        observed_at_[static_cast<std::size_t>(my) * raw_.GetSizeInCellsX() + mx] = stamp;
        return true;
    }

    void RollingObstacleGrid::Expire(double now, double timeout)
    {
        if (!std::isfinite(now) || !std::isfinite(timeout) || timeout <= 0.0) {
            throw std::invalid_argument("Observation expiry requires finite time and positive timeout");
        }
        for (std::size_t index = 0; index < observed_at_.size(); ++index) {
            if (observed_at_[index] != kNeverObserved &&
                (now < observed_at_[index] || now - observed_at_[index] > timeout)) {
                mini_nav_core::MapLocation location{};
                raw_.IndexToMap(index, location);
                raw_.SetCost(location.x, location.y, kUnknown);
                observed_at_[index] = kNeverObserved;
            }
        }
    }

    Costmap2D RollingObstacleGrid::Inflated() const
    {
        return InflateCostmap(raw_, inflation_);
    }

    const Costmap2D & RollingObstacleGrid::Raw() const
    {
        return raw_;
    }
}
