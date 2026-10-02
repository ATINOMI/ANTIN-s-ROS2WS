/**
 * @file costmap_2d.cpp
 * @brief ROS 无关二维代价图、坐标换算和栅格绘制。
 * @author Antinomy
 * @date 2026-10-01
 */
/* Includes ----------------------------------------------------------------*/
#include "mini_nav_core/map/costmap_2d.hpp"

#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{
    /* Constructor ---------------------------------------------------------*/
    // 初始化地图几何信息，并将所有栅格填充为默认代价。
    /**
     * @brief 创建一张指定尺寸和分辨率的代价地图。
     * @param cells_size_x x 方向的栅格数量。
     * @param cells_size_y y 方向的栅格数量。
     * @param resolution 每个栅格边长，单位为米。
     * @param origin_x 地图左下角在世界坐标系中的 x 坐标，单位为米。
     * @param origin_y 地图左下角在世界坐标系中的 y 坐标，单位为米。
     * @param default_value 新建栅格的默认代价值。
     *
     * @throw std::invalid_argument；地图过大时抛出 std::length_error。
     */
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
    /**
     * @brief 调整地图尺寸和坐标信息，并用默认代价值重建全部栅格。
     * @param size_x x 方向的栅格数量。
     * @param size_y y 方向的栅格数量。
     * @param resolution 每个栅格边长，单位为米。
     * @param origin_x 地图左下角的世界 x 坐标，单位为米。
     * @param origin_y 地图左下角的世界 y 坐标，单位为米。
     *
     * @throw std::invalid_argument；地图过大时抛出 std::length_error。
     */
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
    /**
     * @brief 读取指定二维栅格的代价。
     *
     * @param mx x 格下标。
     * @param my y 格下标。
     * @return 0..255 代价。
     * @throws std::out_of_range 格坐标越界。
     */
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
    /**
     * @brief 读取按行存储的一维格索引的代价。
     *
     * @param index 一维存储索引。
     * @return 0..255 代价。
     * @throws std::out_of_range 索引越界。
     */
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
    /**
     * @brief 设置指定地图栅格的代价值。
     * @param mx 栅格的 x 下标。
     * @param my 栅格的 y 下标。
     * @param cost 要写入的代价值。
     *
     * @throw std::out_of_range；mx、my 不在地图范围内时抛出。
     */
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
    /**
     * @brief 用指定代价填充整张地图。
     * @param cost 要写入全部栅格的代价值。
     */
    void Costmap2D::Fill(unsigned char cost)
    {
        costmap_.assign(costmap_.size(), cost);
    }

    // 使用 Bresenham 算法连接两个端点，水平、竖直和斜向线段都适用。
    /**
     * @brief 绘制一条包含起点和终点且四邻域连续的栅格直线。
     * @param start 直线起点的栅格坐标。
     * @param end 直线终点的栅格坐标。
     * @param cost 要写入直线上全部栅格的代价值。
     *
     * @throw std::out_of_range；任一端点不在地图范围内时抛出。
     */
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
    /**
     * @brief 填充两个包含端点围成的矩形区域。
     * @param first 矩形的一个角点。
     * @param second 矩形的另一个角点。
     * @param cost 要写入矩形内全部栅格的代价值。
     *
     * 角点传入顺序不限。
     * @throw std::out_of_range；任一角点不在地图范围内时抛出。
     */
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
    /**
     * @brief 检查给定的栅格坐标是否在地图范围内。
     * @param mx 栅格的 x 下标。
     * @param my 栅格的 y 下标。
     * @return 栅格在地图范围内时返回 true，否则返回 false。
     */
    bool Costmap2D::IsInBounds(unsigned int mx, unsigned int my) const
    {
        return mx < size_x_ && my < size_y_;
    }

    /**
     * @brief 将二维栅格坐标转换为按行存储的一维下标。
     * @param mx 栅格的 x 下标。
     * @param my 栅格的 y 下标。
     * @param index 返回对应的一维下标。
     * @return 坐标在地图范围内时返回 true，否则返回 false。
     *        返回 false 时不修改 index。
     */
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

    /**
     * @brief 将按行存储的一维下标转换为栅格坐标。
     * @param index 一维数组下标。
     * @param location 返回对应的栅格坐标。
     * @return 下标在地图范围内时返回 true，否则返回 false。
     *        返回 false 时不修改 location。
     */
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

    /** @brief 获取地图中栅格的总数。
     * @return 地图总格数。
     */
    std::size_t Costmap2D::GetCellCount() const
    {
        return costmap_.size();
    }

    /*
     将地图栅格坐标转换为世界坐标，输出坐标位于栅格中心。
     加上 0.5，使输出坐标落在栅格中心而不是左下角边界。
    */
    /**
     * @brief 将地图栅格坐标转换为世界坐标。
     * @param mx 栅格的 x 下标。
     * @param my 栅格的 y 下标。
     * @param wx 返回对应栅格中心的世界 x 坐标，单位为米。
     * @param wy 返回对应栅格中心的世界 y 坐标，单位为米。
     * @throw std::out_of_range；mx、my 不在地图范围内时抛出。
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
     将世界坐标转换为包含该点的栅格下标；不输出世界坐标。
     先检查输入坐标是否在地图范围内，再进行浮点运算和类型转换。
    */
    /**
     * @brief 将世界坐标转换为地图栅格坐标。
     * @param wx 世界 x 坐标，单位为米。
     * @param wy 世界 y 坐标，单位为米。
     * @param mx 返回对应栅格的 x 下标。
     * @param my 返回对应栅格的 y 下标。
     * @return 坐标在地图合法范围内时返回 true，否则返回 false。
     *        返回 false 时不修改 mx、my。
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
    /**
     * @brief 查询地图宽度。
     * @return x 方向格数。
     */
    unsigned int Costmap2D::GetSizeInCellsX() const
    {
        return size_x_;
    }

    /**
     * @brief 查询地图高度。
     * @return y 方向格数。
     */
    unsigned int Costmap2D::GetSizeInCellsY() const
    {
        return size_y_;
    }

    //
    /** @brief 获取地图原点的世界 x 坐标，单位为米。
     * @return 地图左下角 x，米。
     */
    double Costmap2D::GetOriginX() const
    {
        return origin_x_;
    }

    /** @brief 获取地图原点的世界 y 坐标，单位为米。
     * @return 地图左下角 y，米。
     */
    double Costmap2D::GetOriginY() const
    {
        return origin_y_;
    }

    /** @brief 获取地图分辨率，即每个栅格边长，单位为米。
     * @return 每格边长，米。
     */
    double Costmap2D::GetResolution() const
    {
        return resolution_;
    }

    /* Internal helpers --------------------------------------------------------*/

    // 构造和重置共用的地图建立流程，集中维护校验、分配和提交规则。
    /**
     * @brief 构造和重置共用的地图建立流程，
     *        集中维护校验、分配和提交规则。
     * @param size_x x 方向的栅格数量。
     * @param size_y y 方向的栅格数量。
     * @param resolution 每个栅格边长，单位为米。
     * @param origin_x 地图左下角的世界 x 坐标，单位为米。
     * @param origin_y 地图左下角的世界 y 坐标，单位为米。
     */
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
    /**
     * @brief 获取栅格坐标在按行存储数组中的下标。
     * @param mx 栅格的 x 下标。
     * @param my 栅格的 y 下标。
     * @return 对应的一维数组下标。
     */
    std::size_t Costmap2D::GetIndex(unsigned int mx, unsigned int my) const
    {
        // 本私有函数不检查边界；访问入口应先验证坐标。
        return static_cast<std::size_t>(my) * size_x_ + mx;
    }

    // 验证地图几何信息的有效性，避免无效尺寸和坐标进入后续计算。
    /**
     * @brief 校验地图几何信息的合法性。
     * @param size_x x 方向的栅格数量。
     * @param size_y y 方向的栅格数量。
     * @param resolution 每个栅格边长，单位为米。
     * @param origin_x 地图左下角的世界 x 坐标，单位为米。
     * @param origin_y 地图左下角的世界 y 坐标，单位为米。
     */
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

        // 分辨率必须为有限正数，保证坐标换算的除法有定义。
        if (!std::isfinite(resolution) || resolution <= 0.0) {
            throw std::invalid_argument("Costmap resolution must be finite and greater than zero");
        }

        // 检查地图原点坐标是否为有限数，避免无效坐标进入后续计算。
        if (!std::isfinite(origin_x) || !std::isfinite(origin_y)) {
            throw std::invalid_argument("Costmap origin must be finite");
        }
    }

    // 计算地图中栅格的总数，并检查乘法溢出和容器容量。
    /**
     * @brief 计算地图中栅格的总数。
     * @param size_x x 方向的栅格数量。
     * @param size_y y 方向的栅格数量。
     * @return 栅格总数。
     */
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
