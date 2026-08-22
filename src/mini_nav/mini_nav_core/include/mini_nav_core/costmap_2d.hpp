#pragma once

/* Includes ----------------------------------------------------------------*/
#include <vector>

/* Namespace ---------------------------------------------------------------*/
namespace mini_nav_core
{
    /* Type definitions --------------------------------------------------------*/
    /// 用于保存地图中一个栅格单元的 x、y 下标。
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
             * 调用者必须保证 mx、my 位于地图范围内。
             */
            unsigned char GetCost(unsigned int mx, unsigned int my) const;

            /**
             * @brief 通过一维存储下标获取栅格的代价值。
             * @param index 按行存储的一维数组下标。
             * @return 该栅格保存的代价值。
             *
             * 调用者必须保证 index 小于地图总栅格数。
             */
            unsigned char GetCost(unsigned int index) const;

            /**
             * @brief 设置指定地图栅格的代价值。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @param cost 要写入的代价值。
             *
             * 调用者必须保证 mx、my 位于地图范围内。
             */
            void SetCost(unsigned int mx, unsigned int my, unsigned char cost);

            /**
             * @brief 将地图栅格坐标转换为世界坐标。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @param wx 返回对应栅格中心的世界 x 坐标，单位为米。
             * @param wy 返回对应栅格中心的世界 y 坐标，单位为米。
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
             */
            bool WorldToMap(double wx,
                            double wy,
                            unsigned int & mx,
                            unsigned int & my) const;

            /** @brief 获取 x 方向的栅格数量。 */
            unsigned int GetSizeInCellsX() const;

            /** @brief 获取 y 方向的栅格数量。 */
            unsigned int GetSizeInCellsY() const;

            /** @brief 获取地图原点的世界 x 坐标，单位为米。 */
            double GetOriginX() const;

            /** @brief 获取地图原点的世界 y 坐标，单位为米。 */
            double GetOriginY() const;

            /** @brief 获取地图分辨率，即每个栅格边长，单位为米。 */
            double GetResolution() const;

        /* Private members ------------------------------------------------------*/
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

            /**
             * @brief 将二维栅格坐标转换为按行存储的一维下标。
             * @param mx 栅格的 x 下标。
             * @param my 栅格的 y 下标。
             * @return costmap_ 中对应的一维下标。
             */
            /* Internal helpers --------------------------------------------------*/
            inline unsigned int GetIndex(unsigned int mx, unsigned int my) const
            {
                return my * size_x_ + mx;
            }
    };
}
