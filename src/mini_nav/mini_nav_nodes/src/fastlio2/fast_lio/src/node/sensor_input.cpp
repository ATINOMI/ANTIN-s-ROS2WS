#include "mini_nav_nodes/fastlio2/fastlio_node.hpp"
#include <pcl_conversions/pcl_conversions.h>

namespace mini_nav_nodes::fastlio2 {
double get_time_sec(const builtin_interfaces::msg::Time &time) { return rclcpp::Time(time).seconds(); }
rclcpp::Time get_ros_time(double timestamp) {
    const auto sec = static_cast<int32_t>(std::floor(timestamp));
    const auto nanosec = static_cast<uint32_t>((timestamp - std::floor(timestamp)) * 1e9);
    return rclcpp::Time(sec, nanosec);
}
ImuSample::Ptr DecodeImu(const sensor_msgs::msg::Imu &message) {
    auto sample = std::make_shared<ImuSample>();
    sample->stamp = get_time_sec(message.header.stamp);
    const auto &a = message.linear_acceleration;
    const auto &w = message.angular_velocity;
    sample->linear_acceleration = {a.x, a.y, a.z};
    sample->angular_velocity = {w.x, w.y, w.z};
    return sample;
}
void SensorInput::process(const sensor_msgs::msg::PointCloud2::UniquePtr &message,
                          PointCloudXYZI::Ptr &output) {
    switch (preprocessing.lidar_type) {
    case OUST64: {
        pcl::PointCloud<ouster_ros::Point> cloud;
        pcl::fromROSMsg(*message, cloud);
        preprocessing.process(cloud, output, get_time_sec(message->header.stamp));
        break;
    }
    case VELO16: {
        pcl::PointCloud<velodyne_ros::Point> cloud;
        pcl::fromROSMsg(*message, cloud);
        preprocessing.process(cloud, output);
        break;
    }
    case MID360: {
        pcl::PointCloud<livox_ros::LivoxPointXyzrtl> cloud;
        pcl::fromROSMsg(*message, cloud);
        preprocessing.process(cloud, output);
        break;
    }
    default: {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*message, cloud);
        preprocessing.process(cloud, output);
        break;
    }
    }
}
void SensorInput::process(const livox_ros_driver2::msg::CustomMsg::UniquePtr &message,
                          PointCloudXYZI::Ptr &output) {
    LivoxScan scan;
    scan.point_num = message->point_num;
    scan.points.reserve(message->points.size());
    for (const auto &p : message->points)
        scan.points.push_back({p.x, p.y, p.z, p.offset_time, p.reflectivity, p.line, p.tag});
    preprocessing.process(scan, output);
}

#define PUBFRAME_PERIOD (20)
void FastlioNode::standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg) {
    if (locate_in_prior_map && !initial_pose_received) {
        return;
    }
    mtx_buffer.lock();
    scan_count++;
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    if (!is_first_lidar && cur_time < last_timestamp_lidar) {
        RCLCPP_ERROR(this->get_logger(), "lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    if (is_first_lidar) {
        is_first_lidar = false;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(cur_time);
    last_timestamp_lidar = cur_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void FastlioNode::livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg) {
    if (locate_in_prior_map && !initial_pose_received) {
        return;
    }
    mtx_buffer.lock();
    double cur_time = get_time_sec(msg->header.stamp);
    double preprocess_start_time = omp_get_wtime();
    scan_count++;
    if (!is_first_lidar && cur_time < last_timestamp_lidar) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        lidar_buffer.clear();
    }
    if (is_first_lidar) {
        is_first_lidar = false;
    }
    last_timestamp_lidar = cur_time;

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() &&
        !lidar_buffer.empty()) {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu,
               last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 &&
        !imu_buffer.empty()) {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void FastlioNode::imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg_in) {
    if (locate_in_prior_map && !initial_pose_received) {
        return;
    }
    publish_count++;
    sensor_msgs::msg::Imu::SharedPtr msg(new sensor_msgs::msg::Imu(*msg_in));

    msg->header.stamp = get_ros_time(get_time_sec(msg_in->header.stamp) - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en) {
        msg->header.stamp = rclcpp::Time(timediff_lidar_wrt_imu + get_time_sec(msg_in->header.stamp));
    }

    double timestamp = get_time_sec(msg->header.stamp);

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu) {
        std::cerr << "lidar loop back, clear buffer" << std::endl;
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(DecodeImu(*msg));
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

bool FastlioNode::sync_packages(MeasureGroup &meas) {
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if (!lidar_pushed) {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            std::cerr << "Too few input point cloud!\n";
        } else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime) {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        } else {
            scan_num++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime +=
                (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time) {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->stamp;
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
        imu_time = imu_buffer.front()->stamp;
        if (imu_time > lidar_end_time)
            break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}
}
