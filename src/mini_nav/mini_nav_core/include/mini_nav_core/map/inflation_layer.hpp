/**
 * @file inflation_layer.hpp
 * @brief 声明静态障碍膨胀参数和规划代价地图生成接口。
 *
 * 本模块只处理 ROS 无关的 Costmap2D。调用方保留原始地图供定位使用，
 * 将返回的独立地图交给规划器；机器人尺寸和安全余量由调用方按实际车型配置。
 * @author Antinomy
 * @date 2026-09-28
 */
#pragma once

#include "mini_nav_core/map/costmap_2d.hpp"

namespace mini_nav_core
{
    /**
     * @brief 静态障碍膨胀所需的几何和衰减参数。
     *
     * robot_radius 与 safety_margin 之和供真实车体安全检查使用；
     * inscribed_radius 可独立指定膨胀硬区，默认零沿用车体安全半径。
     * inflation_radius 按官方规则向上取整到整格，距离从源格中心量起。
     */
    struct InflationParameters
    {
        /// 机器人在平面上的保守外接圆半径，单位：米。
        double robot_radius = 0.24;
        /// 外接圆之外额外保留的安全距离，单位：米，允许为零。
        double safety_margin = 0.02;
        /// 障碍代价传播半径，单位：米，不得小于车体安全半径或膨胀内切半径。
        double inflation_radius = 0.45;
        /// 软代价的指数衰减系数，单位：1/米；越大则离开安全区后衰减越快。
        double cost_scaling_factor = 10.0;
        /// 是否将未知格作为完整膨胀源，对应官方同名参数。
        bool inflate_around_unknown = false;
        /// 膨胀用内切半径，单位：米；零表示沿用 robot_radius + safety_margin。
        double inscribed_radius = 0.0;
    };

    /**
     * @brief 从原始静态图生成独立的膨胀规划图。
     *
     * 对齐 Nav2 1.3.12 InflationLayer 的全图更新：按格中心欧氏距离
     * 分组，四邻域传播且首次访问锁定来源。254 障碍格在硬半径内
     * 写入 253，外圈指数软代价允许截断到零，传播范围为
     * ceil(inflation_radius / resolution) 格。
     * inflate_around_unknown 开启时 255 也作为完整膨胀源。
     * 接收规则固定为官方默认 inflate_unknown=false：未知格可被
     * 253 或 254 覆盖，不接收软代价；其余格与新代价取最大值。
     * 不额外膨胀地图外边界；连续车体扫掠仍由规划器另行检查。
     * 输出沿用输入尺寸、分辨率和原点，原始地图不变。
     *
     * @param source 原始静态代价地图；函数不会修改它。
     * @param parameters 机器人半径、安全余量、膨胀半径和衰减系数。
     * @return 可供规划器使用的独立 Costmap2D。
     * @throws std::invalid_argument 半径或系数非有限值、范围不合法时抛出。
     */
    Costmap2D InflateCostmap(
        const Costmap2D & source,
        const InflationParameters & parameters);
}
