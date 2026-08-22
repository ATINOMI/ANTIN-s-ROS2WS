/* Includes ----------------------------------------------------------------*/
#include "../include/mini_nav_core/costmap_2d.hpp"

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{
    /* Construction and resizing ----------------------------------------------*/
    // 初始化地图几何信息，并将所有栅格填充为默认代价。
    Costmap2D::Costmap2D(
                unsigned int cells_size_x,
                unsigned int cells_size_y,
                double resolution,
                double origin_x,
                double origin_y,
                unsigned char default_value):size_x_(cells_size_x),
                                            size_y_(cells_size_y),
                                            resolution_(resolution),
                                            origin_x_(origin_x),
                                            origin_y_(origin_y),
                                            default_value_(default_value),
                                            costmap_(cells_size_x * cells_size_y,
                                                        default_value)
            {
            }

    // 重建底层数组；旧地图的代价会恢复为 default_value_。
    void Costmap2D::ResizeMap(
                unsigned int size_x,
                unsigned int size_y,
                double resolution,
                double origin_x,
                double origin_y)
    {
        size_x_ = size_x;
        size_y_ = size_y;
        resolution_ = resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        costmap_.assign(size_x * size_y, default_value_);
    }

    /* Cell cost access --------------------------------------------------------*/
    /* Cell cost access --------------------------------------------------------*/
    // 二维坐标先转换为一维下标，再读取对应栅格。
    unsigned char Costmap2D::GetCost(unsigned int mx, unsigned int my) const
    {
        return costmap_[GetIndex(mx, my)];
    }

    // 调用方已持有一维下标时，可以直接读取底层数组。
    unsigned char Costmap2D::GetCost(unsigned int undex) const
    {
        return costmap_[undex];
    }

    // 二维坐标先转换为一维下标，再写入新的代价。
    void Costmap2D::SetCost(unsigned int mx, unsigned int my, unsigned char cost)
    {
        costmap_[GetIndex(mx, my)] = cost;
    }

    /* Coordinate conversion ---------------------------------------------------*/
    /* Coordinate conversion ---------------------------------------------------*/
    // 加上 0.5，使输出坐标落在栅格中心而不是左下角边界。
    void Costmap2D::MapToWorld(unsigned int mx,
                            unsigned int my,
                            double & wx,
                            double & wy) const
    {
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }

    // 先检查下界，避免负数转换为 unsigned int 后得到异常的大数。
    bool Costmap2D::WorldToMap(double wx,
                            double wy,
                            unsigned int & mx,
                            unsigned int & my) const
    {
        if (wx < origin_x_ || wy < origin_y_) {
            return false;
        }

        mx = static_cast<unsigned int>((wx - origin_x_) / resolution_);
        my = static_cast<unsigned int>((wy - origin_y_) / resolution_);

        // 再检查上界，保证计算出的栅格属于当前地图。
        if (mx < size_x_ && my < size_y_) {
            return true;
        }
        return false;
    }

    /* Metadata accessors ------------------------------------------------------*/
    /* Metadata accessors ------------------------------------------------------*/
    unsigned int Costmap2D::GetSizeInCellsX() const
    {
        return size_x_;
    }

    unsigned int Costmap2D::GetSizeInCellsY() const
    {
        return size_y_;
    }

    double Costmap2D::GetOriginX() const
    {
        return origin_x_;
    }

    double Costmap2D::GetOriginY() const
    {
        return origin_y_;
    }

    double Costmap2D::GetResolution() const
    {
        return resolution_;
    }
}
