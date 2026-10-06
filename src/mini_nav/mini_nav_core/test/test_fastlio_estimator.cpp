#include <gtest/gtest.h>
#include <limits>
#include "mini_nav_core/localization/fastlio2/fastlio_estimator.hpp"
#include "mini_nav_core/localization/fastlio2/pointcloud_preprocess.hpp"

namespace {
using namespace mini_nav_core::fastlio2;

PointCloudXYZI::Ptr Room(double offset = 0.0) {
    auto cloud = std::make_shared<PointCloudXYZI>();
    for (int i = -15; i <= 15; ++i) {
        for (int j = -15; j <= 15; ++j) {
            for (int axis = 0; axis < 3; ++axis) {
                PointType p{};
                p.x = axis == 0 ? 3.0f : i * 0.1f;
                p.y = axis == 1 ? 3.0f : (axis == 0 ? i : j) * 0.1f;
                p.z = axis == 2 ? -1.0f : j * 0.1f;
                p.x += offset;
                p.intensity = 10.0f;
                cloud->push_back(p);
            }
        }
    }
    return cloud;
}

MeasureGroup Frame(int index, double offset = 0.0) {
    MeasureGroup frame;
    frame.lidar_beg_time = 10.0 + index * 0.1;
    frame.lidar_end_time = frame.lidar_beg_time + 0.09;
    frame.lidar = Room(offset);
    for (int k = 0; k <= 10; ++k) {
        auto imu = std::make_shared<ImuSample>();
        imu->stamp = frame.lidar_beg_time - 0.01 + k * 0.01;
        imu->linear_acceleration.z = 9.81;
        frame.imu.push_back(imu);
    }
    return frame;
}

FastlioConfig Config() {
    FastlioConfig config;
    config.snapshot_input = true;
    config.estimate_extrinsics = false;
    config.filter_size_surf = config.filter_size_map = 0.15;
    return config;
}

TEST(FastlioEstimator, RejectsIncompleteMeasurementsWithoutStartingInitialization) {
    FastlioEstimator estimator(Config());
    auto frame = Frame(0);
    frame.imu.clear();
    EXPECT_FALSE(estimator.Process(frame).match_available);
    frame = Frame(1);
    frame.lidar.reset();
    EXPECT_FALSE(estimator.Process(frame).valid);
    EXPECT_FALSE(estimator.Process(Frame(2)).match_available);
}

TEST(FastlioEstimator, StationaryRoomAndIndependentInterleavedInstances) {
    FastlioEstimator first(Config()), second(Config());
    for (int i = 0; i < 18; ++i) {
        const auto &a = first.Process(Frame(i));
        const auto &b = second.Process(Frame(i, 0.6));
        if (i < 5)
            continue;
        ASSERT_TRUE(a.match_available);
        ASSERT_TRUE(b.match_available);
        EXPECT_TRUE(a.valid);
        EXPECT_TRUE(b.valid);
        EXPECT_LT(a.pos.norm(), 1e-4);
        EXPECT_LT(b.pos.norm(), 1e-4);
        EXPECT_LT(a.rot.angularDistance(Eigen::Quaterniond::Identity()), 1e-4);
        EXPECT_TRUE(a.covariance.allFinite());
        EXPECT_GT(a.effective_points, 30);
        EXPECT_EQ(a.effective->size(), static_cast<std::size_t>(a.effective_points));
        EXPECT_NE(a.undistorted.get(), b.undistorted.get());
        EXPECT_NEAR(a.stamp, 10.09 + i * 0.1, 1e-10);
    }
}

TEST(FastlioEstimator, InvalidPriorMatchWithholdsValidityAndReportsEmptyEffectiveCloud) {
    auto config = Config();
    config.prior_mode = true;
    FastlioEstimator estimator(config);
    estimator.SetPriorMap(Room(100.0));
    bool checked = false;
    for (int i = 0; i < 12; ++i) {
        const auto &result = estimator.Process(Frame(i));
        if (!result.match_available)
            continue;
        checked = true;
        EXPECT_FALSE(result.valid);
        EXPECT_EQ(result.effective_points, 0);
        EXPECT_TRUE(result.effective->empty());
        EXPECT_TRUE(std::isinf(result.mean_abs_residual));
    }
    EXPECT_TRUE(checked);
}

TEST(FastlioEstimator, MissingLogDirectoryDoesNotCrashEstimationOrDestruction) {
    auto config = Config();
    config.runtime_log = true;
    config.log_root = "/mini_nav_test_nonexistent_directory";
    FastlioEstimator estimator(config);
    for (int i = 0; i < 8; ++i)
        estimator.Process(Frame(i));
    EXPECT_TRUE(estimator.Result().valid);
}

TEST(FastlioPreprocess, SnapshotFiltersBlindAndNonfinitePointsAndKeepsZeroPointTimes) {
    Preprocess preprocessing;
    preprocessing.set(false, GAZEBO_SNAPSHOT, 0.1, 1);
    preprocessing.time_unit = SEC;
    pcl::PointCloud<pcl::PointXYZI> input;
    pcl::PointXYZI p{};
    p.x = 1.0f;
    p.intensity = 42.0f;
    input.push_back(p);
    p.x = 0.05f;
    input.push_back(p);
    p.x = std::numeric_limits<float>::quiet_NaN();
    input.push_back(p);
    PointCloudXYZI::Ptr output(new PointCloudXYZI());
    preprocessing.process(input, output);
    ASSERT_EQ(output->size(), 1u);
    EXPECT_FLOAT_EQ(output->front().x, 1.0f);
    EXPECT_FLOAT_EQ(output->front().intensity, 42.0f);
    EXPECT_FLOAT_EQ(output->front().curvature, 0.0f);
}
}
