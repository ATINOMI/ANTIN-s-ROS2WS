#pragma once

#include <mutex>
#include <condition_variable>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "mini_nav_nodes/fastlio2/sensor_input.hpp"
#include "mini_nav_core/localization/fastlio2/fastlio_estimator.hpp"

namespace mini_nav_nodes::fastlio2 {
using namespace mini_nav_core::fastlio2;
class FastlioNode : public rclcpp::Node {
  public:
    explicit FastlioNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
    void Finish();

  private:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFull_body_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubIkdTree_;
    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pubMatchQuality_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_pcl_pc_;
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr sub_pcl_livox_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_init_pose_;

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr map_pub_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr map_save_srv_;

    std::unique_ptr<FastlioEstimator> estimator_;
    std::shared_ptr<SensorInput> p_pre{new SensorInput()};
    std::mutex mtx_buffer;
    std::condition_variable sig_buffer;
    std::string map_file_path, lid_topic, imu_topic, odom_frame, sensor_frame, base_frame;
    bool send_odom_base_tf = false, locate_in_prior_map = false, initial_pose_received = false;
    bool path_en = true, effect_pub_en = false, map_pub_en = false, ikd_tree_pub_en = false;
    bool scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
    bool runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true;
    std::string prior_map_path;
    double time_diff_lidar_to_imu = 0, timediff_lidar_wrt_imu = 0;
    bool timediff_set_flg = false, is_first_lidar = true, lidar_pushed = false;
    double last_timestamp_lidar = 0, last_timestamp_imu = -1, lidar_mean_scantime = 0, lidar_end_time = 0;
    int scan_num = 0, scan_count = 0, publish_count = 0, pcd_save_interval = -1, NUM_MAX_ITERATIONS = 0;
    double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
    double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0,
           cube_len = 0;
    float DET_RANGE = 300;
    std::vector<double> extrinT, extrinR;
    std::deque<double> time_buffer;
    std::deque<PointCloudXYZI::Ptr> lidar_buffer;
    std::deque<ImuSample::Ptr> imu_buffer;
    MeasureGroup Measures;
    nav_msgs::msg::Path path;
    nav_msgs::msg::Odometry odomAftMapped;
    geometry_msgs::msg::Quaternion geoQuat;
    geometry_msgs::msg::PoseStamped msg_body_pose;
    PointCloudXYZI::Ptr pcl_wait_pub{new PointCloudXYZI()};
    void read_parameters();
    void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::UniquePtr msg);
    void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::UniquePtr msg);
    void imu_cbk(const sensor_msgs::msg::Imu::UniquePtr msg);
    bool sync_packages(MeasureGroup &meas);
    void timer_callback();
    void map_publish_callback();
    void map_save_callback(std_srvs::srv::Trigger::Request::ConstSharedPtr req,
                           std_srvs::srv::Trigger::Response::SharedPtr res);
    void initial_pose_cbk(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    void publish_frame_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
    void publish_frame_body(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
    void publish_effect_world(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
    void publish_map(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
    void publish_ikd_tree(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub);
    void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub);
    void publish_path(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub);
    void save_to_pcd();
    template <typename T> void set_posestamp(T &out);
};
}
