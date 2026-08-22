/* Includes ----------------------------------------------------------------*/
#include "mini_nav_core/costmap_2d.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{
    /* Constructor ---------------------------------------------------------*/
    // 初始化地图几何信息，并将所有栅格填充为默认代价。
    Costmap2D::Costmap2D(
                unsigned int cells_size_x,
                unsigned int cells_size_y,
                double resolution,
                double origin_x,
                double origin_y,
                unsigned char default_value):size_x_(0),
                                             size_y_(0),
                                             resolution_(0.0),
                                             origin_x_(0.0),
                                             origin_y_(0.0),
                                             default_value_(default_value)
    {
        // 构造函数只表达“建立初始地图”的意图，具体工作由私有函数完成。
        InitializeMap(cells_size_x, cells_size_y, resolution, origin_x, origin_y);
    }

/*  Function implementations ----------------------------------------------*/
    // 重建底层数组；旧地图的代价会恢复为 default_value_。
    void Costmap2D::ResizeMap(
                unsigned int size_x,
                unsigned int size_y,
                double resolution,
                double origin_x,
                double origin_y)
    {
        // 重置入口只表达“替换已有地图”的意图。
        InitializeMap(size_x, size_y, resolution, origin_x, origin_y);
    }




    /* Cell cost access --------------------------------------------------------*/
    // 二维坐标先转换为一维下标，再读取对应栅格。
    unsigned char Costmap2D::GetCost(unsigned int mx, unsigned int my) const
    {

        if (!IsInBounds(mx, my)) // 检查栅格坐标是否在地图范围内
        {
            // 如果不在范围内，抛出 std::out_of_range 异常，提示调用方。
            throw std::out_of_range("Costmap cell coordinates are out of bounds");
        }
        // 通过 GetIndex() 获取一维下标，并返回对应栅格的代价值。
        return costmap_[GetIndex(mx, my)];
    }

    // 调用方已持有一维下标时，可以直接读取底层数组。
    unsigned char Costmap2D::GetCost(unsigned int index) const
    {

        // 检查一维下标是否在地图范围内，避免越界访问。
        if (static_cast<std::size_t>(index) >= costmap_.size())
        {
            // 如果下标越界，抛出 std::out_of_range 异常，提示调用方。
            throw std::out_of_range("Costmap cell index is out of bounds");
        }
        return costmap_[index];
    }

    // 二维坐标先转换为一维下标，再写入新的代价。
    void Costmap2D::SetCost(unsigned int mx, unsigned int my, unsigned char cost)
    {
        // 检查栅格坐标是否在地图范围内，避免越界写入。
        if (!IsInBounds(mx, my)) {
            // 如果不在范围内，抛出 std::out_of_range 异常，提示调用方。
            throw std::out_of_range("Costmap cell coordinates are out of bounds");
        }
        costmap_[GetIndex(mx, my)] = cost;
    }


    /* Map drawing -------------------------------------------------------------*/
    // 将全部栅格写为同一代价，适合清空或重置地图。
    void Costmap2D::Fill(unsigned char cost)
    {
        costmap_.assign(costmap_.size(), cost);
    }

    // 使用 Bresenham 算法连接两个端点，水平、竖直和斜向线段都适用。
    void Costmap2D::DrawLine(
        const MapLocation & start,
        const MapLocation & end,
        unsigned char cost)
    {
        // 先验证两个端点，保证失败时地图不会被部分修改。
        if (!IsInBounds(start.x, start.y) || !IsInBounds(end.x, end.y)) {
            throw std::out_of_range("Costmap line endpoints are out of bounds");
        }

        // 使用有符号坐标记录每一步的前进方向，避免无符号减法下溢。
        std::int64_t x = static_cast<std::int64_t>(start.x);
        std::int64_t y = static_cast<std::int64_t>(start.y);
        const std::int64_t end_x = static_cast<std::int64_t>(end.x);
        const std::int64_t end_y = static_cast<std::int64_t>(end.y);
        const std::int64_t delta_x = x < end_x ? end_x - x : x - end_x;
        const std::int64_t delta_y = y < end_y ? end_y - y : y - end_y;
        const std::int64_t step_x = x < end_x ? 1 : -1;
        const std::int64_t step_y = y < end_y ? 1 : -1;
        std::int64_t error = delta_x - delta_y;

        // 每次循环写入当前格子，直到终点也被写入。
        while (true) {
            costmap_[GetIndex(
                static_cast<unsigned int>(x),
                static_cast<unsigned int>(y))] = cost;
            if (x == end_x && y == end_y) {
                break;
            }

            const std::int64_t double_error = 2 * error;
            const bool move_x = double_error > -delta_y;
            const bool move_y = double_error < delta_x;

            // 斜向跨步会使两个障碍格只在角点相接；补一个桥接格以保持四邻域连续。
            if (move_x && move_y) {
                costmap_[GetIndex(
                    static_cast<unsigned int>(x + step_x),
                    static_cast<unsigned int>(y))] = cost;
            }
            if (move_x) {
                error -= delta_y;
                x += step_x;
            }
            if (move_y) {
                error += delta_x;
                y += step_y;
            }
        }
    }

    // 填充由两个角点界定的所有栅格，角点传入顺序不影响结果。
    void Costmap2D::FillRectangle(
        const MapLocation & first,
        const MapLocation & second,
        unsigned char cost)
    {
        if (!IsInBounds(first.x, first.y) || !IsInBounds(second.x, second.y)) {
            throw std::out_of_range("Costmap rectangle corners are out of bounds");
        }

        // 先规范化角点，再按行写入包含边界的矩形区域。
        const unsigned int min_x = first.x < second.x ? first.x : second.x;
        const unsigned int max_x = first.x < second.x ? second.x : first.x;
        const unsigned int min_y = first.y < second.y ? first.y : second.y;
        const unsigned int max_y = first.y < second.y ? second.y : first.y;

        for (unsigned int my = min_y; ; ++my) {
            for (unsigned int mx = min_x; ; ++mx) {
                costmap_[GetIndex(mx, my)] = cost;
                if (mx == max_x) {
                    break;
                }
            }
            if (my == max_y) {
                break;
            }
        }
    }

    /* Coordinate and index queries ------------------------------------------*/
    // 统一坐标边界判断，供访问、绘制和查询接口复用。
    bool Costmap2D::IsInBounds(unsigned int mx, unsigned int my) const
    {
        return mx < size_x_ && my < size_y_;
    }

    bool Costmap2D::MapToIndex(
        unsigned int mx,
        unsigned int my,
        std::size_t & index) const
    {
        if (!IsInBounds(mx, my)) {
            return false;
        }

        // 仅在坐标有效时提交输出，失败时保留调用方的原值。
        index = GetIndex(mx, my);
        return true;
    }

    bool Costmap2D::IndexToMap(std::size_t index, MapLocation & location) const
    {
        if (index >= costmap_.size()) {
            return false;
        }

        // 一维下标按行存储：商为 y，余数为 x。
        location.x = static_cast<unsigned int>(index % size_x_);
        location.y = static_cast<unsigned int>(index / size_x_);
        return true;
    }

    std::size_t Costmap2D::GetCellCount() const
    {
        return costmap_.size();
    }

    /*
     将地图栅格坐标转换为世界坐标，输出坐标位于栅格中心。
     加上 0.5，使输出坐标落在栅格中心而不是左下角边界。
    */
    void Costmap2D::MapToWorld(unsigned int mx,
                            unsigned int my,
                            double & wx,
                            double & wy) const
    {
        // 检查栅格坐标是否在地图范围内，避免计算无效的世界坐标。
        if (!IsInBounds(mx, my)) {
            throw std::out_of_range("Costmap cell coordinates are out of bounds");
        }

        // 将栅格坐标转换为世界坐标，输出坐标位于栅格中心。
        wx = origin_x_ + (mx + 0.5) * resolution_;
        wy = origin_y_ + (my + 0.5) * resolution_;
    }

    /*
     将世界坐标转换为地图栅格坐标，输出坐标为栅格左下角。
     先检查输入坐标是否在地图范围内，再进行浮点运算和类型转换。
    */
    bool Costmap2D::WorldToMap(double wx,
                            double wy,
                            unsigned int & mx,
                            unsigned int & my) const
    {
        // 检查输入坐标是否为有限数，并且不小于地图左下角坐标。
        if (!std::isfinite(wx) || !std::isfinite(wy) ||
            wx < origin_x_ || wy < origin_y_) {
            return false;
        }

        // 将世界坐标转换为栅格坐标，使用浮点数计算避免整数除法截断。
        const double map_x = (wx - origin_x_) / resolution_;
        const double map_y = (wy - origin_y_) / resolution_;

        // 先检查计算结果，再转换为无符号整数，避免非法浮点转换。
        if (!std::isfinite(map_x) || !std::isfinite(map_y) ||
            map_x < 0.0 || map_y < 0.0 ||
            map_x >= static_cast<double>(size_x_) ||
            map_y >= static_cast<double>(size_y_)) {
            // 如果计算结果不在地图范围内，返回 false，调用方可据此处理。
            return false;
        }

        // 将浮点数转换为无符号整数，输出栅格坐标。
        mx = static_cast<unsigned int>(map_x);
        my = static_cast<unsigned int>(map_y);
        return true;
    }

    /*
    *  GetSizeInCellsX()        |   GetSizeInCellsY()
    *  返回地图在 x 方向的栅格数量  |   返回地图在 y 方向的栅格数量
    * --------------------------|-----------------------
    *  GetOriginX()             |   GetOriginY()
    *  返回地图原点在 x 方向的坐标  |   返回地图原点在 y 方向的坐标
    *
    */
    unsigned int Costmap2D::GetSizeInCellsX() const
    {
        return size_x_;
    }

    unsigned int Costmap2D::GetSizeInCellsY() const
    {
        return size_y_;
    }

    //
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

    /* Internal helpers --------------------------------------------------------*/

    // 构造和重置共用的地图建立流程，集中维护校验、分配和提交规则。
    void Costmap2D::InitializeMap(
        unsigned int size_x,
        unsigned int size_y,
        double resolution,
        double origin_x,
        double origin_y)
    {
        // 先校验地图几何信息，避免无效尺寸和坐标进入后续计算。
        ValidateGeometry(size_x, size_y, resolution, origin_x, origin_y);
        // 在分配前计算总格数，并检查乘法溢出和容器容量。
        const std::size_t cell_count = CalculateCellCount(size_x, size_y);

        // 先创建新地图。若分配失败，当前地图的元数据和栅格数据保持不变。
        std::vector<unsigned char> new_costmap(cell_count, default_value_);

        // 新栅格数组创建成功后，再一次性提交新的几何信息和地图数据。
        size_x_ = size_x;
        size_y_ = size_y;
        resolution_ = resolution;
        origin_x_ = origin_x;
        origin_y_ = origin_y;
        // 交换新旧数组，避免不必要的拷贝。
        costmap_.swap(new_costmap);
    }

    // 将二维栅格坐标转换为一维数组下标，按行存储。
    std::size_t Costmap2D::GetIndex(unsigned int mx, unsigned int my) const
    {
        // 检查栅格坐标是否在地图范围内，避免越界访问。
        return static_cast<std::size_t>(my) * size_x_ + mx;
    }

    // 验证地图几何信息的有效性，避免无效尺寸和坐标进入后续计算。
    void Costmap2D::ValidateGeometry(
        unsigned int size_x,
        unsigned int size_y,
        double resolution,
        double origin_x,
        double origin_y)
    {
        // 检查地图尺寸是否大于零，避免创建空地图。
        if (size_x == 0 || size_y == 0) {
            throw std::invalid_argument("Costmap dimensions must be greater than zero");
        }

        // 检查地图尺寸是否超过 std::size_t 的最大值，避免溢出。
        if (!std::isfinite(resolution) || resolution <= 0.0) {
            throw std::invalid_argument("Costmap resolution must be finite and greater than zero");
        }

        // 检查地图原点坐标是否为有限数，避免无效坐标进入后续计算。
        if (!std::isfinite(origin_x) || !std::isfinite(origin_y)) {
            throw std::invalid_argument("Costmap origin must be finite");
        }
    }

    // 计算地图中栅格的总数，并检查乘法溢出和容器容量。
    std::size_t Costmap2D::CalculateCellCount(unsigned int size_x, unsigned int size_y)
    {
        // 将 size_x 和 size_y 转换为 std::size_t 类型，避免乘法溢出。
        const std::size_t cells_x = static_cast<std::size_t>(size_x);
        const std::size_t cells_y = static_cast<std::size_t>(size_y);

        // 检查乘法是否会溢出 std::size_t 的最大值，避免无效的栅格总数。
        if (cells_x > std::numeric_limits<std::size_t>::max() / cells_y) {
            throw std::length_error("Costmap cell count overflows size_t");
        }

        // 计算总栅格数，并检查是否超过 std::vector<unsigned char> 的最大容量。
        const std::size_t cell_count = cells_x * cells_y;
        if (cell_count > std::vector<unsigned char>().max_size()) {
            throw std::length_error("Costmap is too large to allocate");
        }

        // 返回计算得到的总栅格数，供后续分配和初始化使用。
        return cell_count;
    }
}
