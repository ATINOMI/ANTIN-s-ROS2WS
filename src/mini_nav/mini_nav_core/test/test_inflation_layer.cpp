/**
 * @file test_inflation_layer.cpp
 * @brief 用已安装 Nav2 膨胀库逐格验证自研膨胀结果。
 */
#include <gtest/gtest.h>

#include <cmath>
#include <fstream>
#include <limits>
#include <random>
#include <string>
#include <vector>

#include "mini_nav_core/map/inflation_layer.hpp"
#include "nav2_costmap_2d/inflation_layer.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "mini_nav_core/navigator/astar_navigator.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"

namespace
{
    using mini_nav_core::Costmap2D;
    using mini_nav_core::InflationParameters;

    class InflationLayer : public ::testing::Test
    {
    protected:
        static void SetUpTestSuite()
        {
            rclcpp::init(0, nullptr);
        }

        static void TearDownTestSuite()
        {
            rclcpp::shutdown();
        }

        // 足迹内切半径精确设为自研硬半径，只对比膨胀核，不混入圆足迹离散误差。
        std::vector<unsigned char> Official(
            const Costmap2D & source, const InflationParameters & parameters,
            const std::vector<geometry_msgs::msg::Point> & reference_footprint = {})
        {
            rclcpp::NodeOptions options;
            options.parameter_overrides({
                rclcpp::Parameter("inflation.inflation_radius", parameters.inflation_radius),
                rclcpp::Parameter("inflation.cost_scaling_factor", parameters.cost_scaling_factor),
                rclcpp::Parameter("inflation.inflate_around_unknown", parameters.inflate_around_unknown)});
            auto node = std::make_shared<nav2_util::LifecycleNode>("inflation_equivalence", "", options);
            tf2_ros::Buffer tf(node->get_clock());
            nav2_costmap_2d::LayeredCostmap layers("map", false, true);
            const auto width = source.GetSizeInCellsX();
            const auto height = source.GetSizeInCellsY();
            layers.resizeMap(width, height, source.GetResolution(), source.GetOriginX(), source.GetOriginY());
            auto layer = std::make_shared<nav2_costmap_2d::InflationLayer>();
            layer->initialize(&layers, "inflation", &tf, node, nullptr);
            layers.addPlugin(layer);
            const double radius = parameters.inscribed_radius > 0.0 ? parameters.inscribed_radius :
                parameters.robot_radius + parameters.safety_margin;
            std::vector<geometry_msgs::msg::Point> footprint;
            for (const auto & xy : std::vector<std::pair<double, double>>{
                    {radius, radius}, {radius, -radius}, {-radius, -radius}, {-radius, radius}}) {
                geometry_msgs::msg::Point point;
                point.x = xy.first;
                point.y = xy.second;
                footprint.push_back(point);
            }
            layers.setFootprint(reference_footprint.empty() ? footprint : reference_footprint);
            auto master = layers.getCostmap();
            for (unsigned int y = 0; y < height; ++y) {
                for (unsigned int x = 0; x < width; ++x) {
                    master->setCost(x, y, source.GetCost(x, y));
                }
            }
            layer->updateCosts(*master, 0, 0, width, height);
            return {master->getCharMap(), master->getCharMap() + source.GetCellCount()};
        }

        void ExpectOfficial(const Costmap2D & source, const InflationParameters & parameters,
            const std::vector<geometry_msgs::msg::Point> & reference_footprint = {})
        {
            const auto expected = Official(source, parameters, reference_footprint);
            const auto actual = mini_nav_core::InflateCostmap(source, parameters);
            unsigned int differences = 0;
            for (unsigned int index = 0; index < expected.size(); ++index) {
                if (actual.GetCost(index) != expected[index]) {
                    if (differences < 5) {
                        ADD_FAILURE() << "cell " << index % source.GetSizeInCellsX() << ","
                                      << index / source.GetSizeInCellsX() << ": actual="
                                      << int(actual.GetCost(index)) << ", Nav2=" << int(expected[index]);
                    }
                    ++differences;
                }
            }
            EXPECT_EQ(differences, 0u);
        }
    };
}

TEST_F(InflationLayer, MatchesSingleObstacleAndFractionalOuterRadius)
{
    Costmap2D source(31, 29, 0.05, -0.5, -0.7, 0);
    source.SetCost(15, 14, 254);
    InflationParameters parameters;
    ExpectOfficial(source, parameters);
    parameters.inflation_radius = 0.461;
    ExpectOfficial(source, parameters);
    // 官方向上取整到 10 格，因此中心距 0.50 m 的格仍接收软代价。
    EXPECT_GT(Official(source, parameters)[14 * 31 + 25], 0);
}

TEST_F(InflationLayer, MatchesEmptyMapAndMapEdgeObstacles)
{
    Costmap2D source(19, 13, 0.05, 0.0, 0.0, 0);
    ExpectOfficial(source, {});
    for (const auto & cell : std::vector<std::pair<unsigned int, unsigned int>>{
            {0, 0}, {18, 0}, {0, 12}, {18, 12}, {9, 0}}) {
        source.SetCost(cell.first, cell.second, 254);
    }
    ExpectOfficial(source, {});
    // learning 的真实圆足迹经官方 padding 后计算内切半径，不使用等半径方形替代。
    InflationParameters learning;
    learning.inscribed_radius = 0.22549849949589046;
    learning.inflation_radius = 0.70;
    learning.cost_scaling_factor = 3.0;
    auto footprint = nav2_costmap_2d::makeFootprintFromRadius(0.22);
    nav2_costmap_2d::padFootprint(footprint, static_cast<double>(0.01F));
    EXPECT_DOUBLE_EQ(nav2_costmap_2d::calculateMinAndMaxDistances(footprint).first,
        learning.inscribed_radius);
    ExpectOfficial(source, learning, footprint);
}

TEST_F(InflationLayer, SeparateInscribedRadiusDoesNotRelaxBodySweep)
{
    Costmap2D source(31, 29, 0.05, 0.0, 0.0, 0);
    source.SetCost(15, 14, 254);
    InflationParameters parameters;
    parameters.inscribed_radius = 0.22549849949589046;
    parameters.inflation_radius = 0.70;
    parameters.cost_scaling_factor = 3.0;
    const auto planning = mini_nav_core::InflateCostmap(source, parameters);
    EXPECT_LT(planning.GetCost(20, 14), 253);
    auto safety_parameters = parameters;
    safety_parameters.inscribed_radius = 0.0;
    const auto safety = mini_nav_core::InflateCostmap(source, safety_parameters);
    EXPECT_EQ(safety.GetCost(20, 14), 253);
    mini_nav_core::AStarPlanner planner;
    // 起点在内切硬圈之外，但 0.26 m 车体仍碰障碍，真实扫掠必须拒绝。
    EXPECT_TRUE(planner.Plan(planning, source,
        parameters.robot_radius + parameters.safety_margin, {20, 14}, {25, 14}, false, 0.0).empty());
}

TEST_F(InflationLayer, MatchesUnknownSourcesAndUnknownReceivers)
{
    Costmap2D source(27, 25, 0.05, 0.0, 0.0, 0);
    source.SetCost(10, 10, 254);
    source.SetCost(11, 10, 255);
    source.SetCost(18, 18, 255);
    InflationParameters parameters;
    ExpectOfficial(source, parameters);
    EXPECT_EQ(Official(source, parameters)[10 * 27 + 11], 253);
    parameters.inflate_around_unknown = true;
    ExpectOfficial(source, parameters);
    EXPECT_GT(Official(source, parameters)[18 * 27 + 24], 0);
}

TEST_F(InflationLayer, MatchesZeroCostAfterExponentialUnderflow)
{
    Costmap2D source(31, 29, 0.05, 0.0, 0.0, 0);
    source.SetCost(15, 14, 254);
    InflationParameters parameters;
    parameters.cost_scaling_factor = 10000.0;
    ExpectOfficial(source, parameters);
}

TEST_F(InflationLayer, MatchesDeterministicMixedMaps)
{
    std::mt19937 random(20261005);
    for (int trial = 0; trial < 24; ++trial) {
        SCOPED_TRACE(trial);
        Costmap2D source(19 + trial % 5, 17 + trial % 7,
                         trial % 2 == 0 ? 0.05 : 0.1, -0.3, -0.7, 0);
        for (unsigned int y = 0; y < source.GetSizeInCellsY(); ++y) {
            for (unsigned int x = 0; x < source.GetSizeInCellsX(); ++x) {
                const unsigned int draw = random() % 40;
                if (draw < 4) {
                    source.SetCost(x, y, draw == 0 ? 255 : 254);
                } else if (draw < 8) {
                    source.SetCost(x, y, static_cast<unsigned char>(random() % 254));
                }
            }
        }
        InflationParameters parameters;
        parameters.robot_radius = trial % 3 == 0 ? 0.10 : 0.24;
        parameters.inflation_radius = trial % 2 == 0 ? 0.461 : 0.73;
        parameters.cost_scaling_factor = trial % 3 == 0 ? 3.0 : 10.0;
        parameters.inflate_around_unknown = trial % 2 != 0;
        parameters.inscribed_radius = trial % 4 == 0 ? 0.22549849949589046 : 0.0;
        ExpectOfficial(source, parameters);
    }
}

TEST_F(InflationLayer, MatchesDefaultMapEveryCell)
{
    // 仓库默认地图是固定 P5/8-bit 栅格；测试直接读取它，避免使用历史实验快照。
    std::ifstream input(MINI_NAV_TEST_MAP, std::ios::binary);
    ASSERT_TRUE(input);
    std::string token;
    auto next_token = [&]() {
        while (input >> token) {
            if (!token.empty() && token.front() == '#') {
                input.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            } else {
                return token;
            }
        }
        return std::string{};
    };
    ASSERT_EQ(next_token(), "P5");
    const auto width = std::stoul(next_token());
    const auto height = std::stoul(next_token());
    ASSERT_EQ(next_token(), "255");
    input.get();
    Costmap2D source(width, height, 0.05, -0.961, -2.072, 0);
    for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
            unsigned char pixel;
            input.read(reinterpret_cast<char *>(&pixel), 1);
            ASSERT_TRUE(input);
            const double occupancy = 1.0 - double(pixel) / 255.0;
            source.SetCost(x, height - 1 - y, occupancy > 0.65 ? 254 : (occupancy < 0.196 ? 0 : 255));
        }
    }
    ExpectOfficial(source, {});
    InflationParameters learning;
    learning.inscribed_radius = 0.22549849949589046;
    learning.inflation_radius = 0.70;
    learning.cost_scaling_factor = 3.0;
    auto footprint = nav2_costmap_2d::makeFootprintFromRadius(0.22);
    nav2_costmap_2d::padFootprint(footprint, static_cast<double>(0.01F));
    ExpectOfficial(source, learning, footprint);
}

TEST_F(InflationLayer, PreservesInputAndRejectsInvalidParameters)
{
    Costmap2D source(7, 5, 0.1, -0.3, -0.7, 0);
    source.SetCost(3, 2, 254);
    const auto result = mini_nav_core::InflateCostmap(source, {});
    EXPECT_EQ(source.GetCost(2, 2), 0);
    EXPECT_EQ(source.GetCost(3, 2), 254);
    EXPECT_EQ(result.GetSizeInCellsX(), source.GetSizeInCellsX());
    EXPECT_DOUBLE_EQ(result.GetOriginX(), source.GetOriginX());
    for (double radius : {-1.0, std::numeric_limits<double>::infinity(),
                          std::numeric_limits<double>::quiet_NaN()}) {
        InflationParameters parameters;
        parameters.inflation_radius = radius;
        EXPECT_THROW(mini_nav_core::InflateCostmap(source, parameters), std::invalid_argument);
        parameters.inflation_radius = 0.70;
        parameters.inscribed_radius = radius;
        EXPECT_THROW(mini_nav_core::InflateCostmap(source, parameters), std::invalid_argument);
    }
    InflationParameters parameters;
    parameters.inscribed_radius = 0.80;
    EXPECT_THROW(mini_nav_core::InflateCostmap(source, parameters), std::invalid_argument);
}
