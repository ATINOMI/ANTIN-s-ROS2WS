/**
 * @file costmap_2d.hpp
 * @brief ROS 无关二维代价图、坐标换算和栅格绘制。
 * @author Antinomy
 * @date 2026-10-01
 */
#pragma once

/* Includes ----------------------------------------------------------------*/
#include <cstddef>
#include <vector>

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{
    /* Type definitions --------------------------------------------------------*/
    /// 用于保存地图中一个栅格单元的 x、y 下标。
    /**
     * @brief 地图单元的无符号 x、y 下标，不是米制坐标。
     */
    struct MapLocation
    {
        unsigned int x;
        unsigned int y;
    };

    /**
     * @class Costmap2D
     * @brief 为最小导航系统保存二维静态代价地图。
     *
     * 地图使用一维 vector 按行存储，每个栅格由 unsigned char 表示其代价。
     * 坐标转换接口负责在世界坐标（米）和地图栅格坐标之间转换。
     */
    /* Class definition --------------------------------------------------------*/
    class Costmap2D
    {
        // 如测试需要，可由专用测试辅助类访问私有成员。
        friend class CostmapTester;

        /* Public API -----------------------------------------------------------*/
        public:
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
            Costmap2D(
                unsigned int cells_size_x,
                unsigned int cells_size_y,
                double resolution,
                double origin_x,
                double origin_y,
                unsigned char default_value = 0);

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
            void ResizeMap(
                unsigned int size_x,
                unsigned int size_y,
                double resolution,
                double origin_x,
                double origin_y);

            /**
             * @brief 获取指定地图栅格的代价值。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @return 该栅格保存的代价值。
             *
             * @throw std::out_of_range；mx、my 不在地图范围内时抛出。
             */
            unsigned char GetCost(unsigned int mx, unsigned int my) const;

            /**
             * @brief 通过一维存储下标获取栅格的代价值。
             * @param index 按行存储的一维数组下标。
             * @return 该栅格保存的代价值。
             *
             * @throw std::out_of_range；index 不在地图范围内时抛出。
             */
            unsigned char GetCost(unsigned int index) const;

            /**
             * @brief 设置指定地图栅格的代价值。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @param cost 要写入的代价值。
             *
             * @throw std::out_of_range；mx、my 不在地图范围内时抛出。
             */
            void SetCost(unsigned int mx, unsigned int my, unsigned char cost);

            /**
             * @brief 用指定代价填充整张地图。
             * @param cost 要写入全部栅格的代价值。
             */
            void Fill(unsigned char cost);

            /**
             * @brief 绘制一条包含起点和终点且四邻域连续的栅格直线。
             * @param start 直线起点的栅格坐标。
             * @param end 直线终点的栅格坐标。
             * @param cost 要写入直线上全部栅格的代价值。
             *
             * @throw std::out_of_range；任一端点不在地图范围内时抛出。
             */
            void DrawLine(
                const MapLocation & start,
                const MapLocation & end,
                unsigned char cost);

            /**
             * @brief 填充两个包含端点围成的矩形区域。
             * @param first 矩形的一个角点。
             * @param second 矩形的另一个角点。
             * @param cost 要写入矩形内全部栅格的代价值。
             *
             * 角点传入顺序不限。
             * @throw std::out_of_range；任一角点不在地图范围内时抛出。
             */
            void FillRectangle(
                const MapLocation & first,
                const MapLocation & second,
                unsigned char cost);

            /**
             * @brief 检查给定的栅格坐标是否在地图范围内。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @return 栅格在地图范围内时返回 true，否则返回 false。
             */
            bool IsInBounds(unsigned int mx, unsigned int my) const;

            /**
             * @brief 将二维栅格坐标转换为按行存储的一维下标。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @param index 返回对应的一维下标。
             * @return 坐标在地图范围内时返回 true，否则返回 false。
             *        返回 false 时不修改 index。
             */
            bool MapToIndex(
                unsigned int mx,
                unsigned int my,
                std::size_t & index) const;

            /**
             * @brief 将按行存储的一维下标转换为栅格坐标。
             * @param index 一维数组下标。
             * @param location 返回对应的栅格坐标。
             * @return 下标在地图范围内时返回 true，否则返回 false。
             *        返回 false 时不修改 location。
             */
            bool IndexToMap(std::size_t index, MapLocation & location) const;

            /** @brief 获取地图中栅格的总数。
             * @return 地图总格数。
             */
            std::size_t GetCellCount() const;

            /**
             * @brief 将地图栅格坐标转换为世界坐标。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @param wx 返回对应栅格中心的世界 x 坐标，单位为米。
             * @param wy 返回对应栅格中心的世界 y 坐标，单位为米。
             * @throw std::out_of_range；mx、my 不在地图范围内时抛出。
             */
            void MapToWorld(unsigned int mx,
                            unsigned int my,
                            double & wx,
                            double & wy) const;

            /**
             * @brief 将世界坐标转换为地图栅格坐标。
             * @param wx 世界 x 坐标，单位为米。
             * @param wy 世界 y 坐标，单位为米。
             * @param mx 返回对应栅格的 x 下标。
             * @param my 返回对应栅格的 y 下标。
             * @return 坐标在地图合法范围内时返回 true，否则返回 false。
             *        返回 false 时不修改 mx、my。
             */
            bool WorldToMap(double wx,
                            double wy,
                            unsigned int & mx,
                            unsigned int & my) const;

            /** @brief 获取 x 方向的栅格数量。 */
            unsigned int GetSizeInCellsX() const;

            /** @brief 获取 y 方向的栅格数量。 */
            unsigned int GetSizeInCellsY() const;

            /** @brief 获取地图原点的世界 x 坐标，单位为米。
             * @return 地图左下角 x，米。
             */
            double GetOriginX() const;

            /** @brief 获取地图原点的世界 y 坐标，单位为米。
             * @return 地图左下角 y，米。
             */
            double GetOriginY() const;

            /** @brief 获取地图分辨率，即每个栅格边长，单位为米。
             * @return 每格边长，米。
             */
            double GetResolution() const;

        /* Private members ------------------------------------------------------*/
        private:
            /// x 方向的栅格数量。
            unsigned int size_x_;
            /// y 方向的栅格数量。
            unsigned int size_y_;
            /// 每个栅格边长，单位为米。
            double resolution_;
            /// 地图左下角的世界 x 坐标，单位为米。
            double origin_x_;
            /// 地图左下角的世界 y 坐标，单位为米。
            double origin_y_;
            /// 创建或重置地图时使用的默认代价值。
            unsigned char default_value_;
            /// 按行存储的代价数组，索引公式为 my * size_x_ + mx。
            std::vector<unsigned char> costmap_;

            /* Internal helpers --------------------------------------------------*/

            /**
             * @brief 构造和重置共用的地图建立流程，
             *        集中维护校验、分配和提交规则。
             * @param size_x x 方向的栅格数量。
             * @param size_y y 方向的栅格数量。
             * @param resolution 每个栅格边长，单位为米。
             * @param origin_x 地图左下角的世界 x 坐标，单位为米。
             * @param origin_y 地图左下角的世界 y 坐标，单位为米。
             */
            void InitializeMap(
                unsigned int size_x,
                unsigned int size_y,
                double resolution,
                double origin_x,
                double origin_y);

            /**
             * @brief 获取栅格坐标在按行存储数组中的下标。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @return 对应的一维数组下标。
             */
            std::size_t GetIndex(unsigned int mx, unsigned int my) const;

            /**
             * @brief 校验地图几何信息的合法性。
             * @param size_x x 方向的栅格数量。
             * @param size_y y 方向的栅格数量。
             * @param resolution 每个栅格边长，单位为米。
             * @param origin_x 地图左下角的世界 x 坐标，单位为米。
             * @param origin_y 地图左下角的世界 y 坐标，单位为米。
             */
            static void ValidateGeometry(
                unsigned int size_x,
                unsigned int size_y,
                double resolution,
                double origin_x,
                double origin_y);

            /**
             * @brief 计算地图中栅格的总数。
             * @param size_x x 方向的栅格数量。
             * @param size_y y 方向的栅格数量。
             * @return 栅格总数。
             */
            static std::size_t CalculateCellCount(
                unsigned int size_x,
                unsigned int size_y);
    };
}
