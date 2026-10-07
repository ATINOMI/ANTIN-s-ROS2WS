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
     * robot_radius 与 safety_margin 之和是不可通行区半径；
     * inflation_radius 是确定障碍物软代价影响的最远距离。硬安全距离
     * 从禁行格面积或地图外边界量起，需结合车体碰撞包络选取。
     */
    struct InflationParameters
    {
        /// 机器人在平面上的保守外接圆半径，单位：米。
        double robot_radius = 0.24;
        /// 外接圆之外额外保留的安全距离，单位：米，允许为零。
        double safety_margin = 0.02;
        /// 障碍代价向外传播的最大半径，单位：米，不得小于硬安全半径。
        double inflation_radius = 0.45;
        /// 软代价的指数衰减系数，单位：1/米；越大则离开安全区后衰减越快。
        double cost_scaling_factor = 10.0;
        /// 是否让未知格向已知区域传播硬安全区；未知格本身始终禁行。
        bool inflate_around_unknown = false;
    };

    /**
     * @brief 从原始静态图生成独立的膨胀规划图。
     *
     * 254 障碍格在硬安全半径内写入 253，在硬半径外至
     * inflation_radius 内写入 1..252 的指数衰减软代价。255 未知格
     * 仅在 inflate_around_unknown 开启时传播硬安全区，从不传播软代价；
     * 地图外边界始终保留硬安全距离。
     * 距离取目标格中心到源格方形面积或地图外边界的最短距离。
     * 栅格间连续移动的车体扫掠由规划器另外检查；源格保持原值。
     * 新代价与原格代价取较大值，因此不会降低已有代价；255 未知格
     * 保持未知。输出沿用输入地图的尺寸、分辨率和原点。
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
