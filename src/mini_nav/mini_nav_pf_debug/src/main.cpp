#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "mini_nav_core/localization/amcl/beam_model.hpp"
#include "mini_nav_core/localization/amcl/differential_motion_model.hpp"
#include "mini_nav_core/localization/amcl/likelihood_field_model.hpp"
#include "mini_nav_core/localization/amcl/particle_filter.hpp"
#include "mini_nav_core/map/costmap_2d.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/exceptions.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace
{

namespace loc = mini_nav_core::localization;

std::string StripLeadingSlash(std::string value)
{
  while (!value.empty() && value.front() == '/') {
    value.erase(value.begin());
  }
  return value;
}

void WriteNumber(std::ostream & out, double value)
{
  if (std::isfinite(value)) {
    out << value;
  } else {
    out << "null";
  }
}

struct Snapshot
{
  std::string stage;
  std::size_t update{0};
  double stamp{0.0};
  std::vector<loc::Particle> particles;
  std::optional<loc::PoseEstimate> estimate;
  std::optional<loc::ResampleTrace> resample;
};

struct Session
{
  std::size_t id{0};
  double initial_stamp{0.0};
  loc::Pose2D initial_pose;
  nav_msgs::msg::OccupancyGrid map;
  std::vector<Snapshot> frames;
};

class PfDebugNode final : public rclcpp::Node
{
public:
  PfDebugNode()
  : rclcpp::Node("pf_debug")
  {
    global_frame_ = StripLeadingSlash(declare_parameter<std::string>("global_frame_id", "map"));
    odom_frame_ = StripLeadingSlash(declare_parameter<std::string>("odom_frame_id", "odom"));
    base_frame_ = StripLeadingSlash(declare_parameter<std::string>("base_frame_id", "base_footprint"));
    map_topic_ = declare_parameter<std::string>("map_topic", "map");
    scan_topic_ = declare_parameter<std::string>("scan_topic", "scan");
    first_map_only_ = declare_parameter<bool>("first_map_only", true);
    update_min_d_ = declare_parameter<double>("update_min_d", 0.25);
    update_min_a_ = declare_parameter<double>("update_min_a", 0.2);
    max_distance_ = declare_parameter<double>("laser_likelihood_max_dist", 2.0);
    max_range_override_ = declare_parameter<double>("laser_max_range", 100.0);
    min_range_override_ = declare_parameter<double>("laser_min_range", -1.0);
    max_updates_ = declare_parameter<int>("max_updates", 15);
    max_sessions_ = declare_parameter<int>("max_sessions", 5);
    output_file_ = declare_parameter<std::string>(
      "output_file", "/tmp/mini_nav_pf_debug.html");
    const auto model_type = declare_parameter<std::string>(
      "laser_model_type", "likelihood_field");

    const double alpha1 = declare_parameter<double>("alpha1", 0.2);
    const double alpha2 = declare_parameter<double>("alpha2", 0.2);
    const double alpha3 = declare_parameter<double>("alpha3", 0.2);
    const double alpha4 = declare_parameter<double>("alpha4", 0.2);
    const double alpha5 = declare_parameter<double>("alpha5", 0.2);
    const double z_hit = declare_parameter<double>("z_hit", 0.5);
    const double z_short = declare_parameter<double>("z_short", 0.05);
    const double z_max = declare_parameter<double>("z_max", 0.05);
    const double z_rand = declare_parameter<double>("z_rand", 0.5);
    const double sigma_hit = declare_parameter<double>("sigma_hit", 0.2);
    const double lambda_short = declare_parameter<double>("lambda_short", 0.1);
    const int max_beams = declare_parameter<int>("max_beams", 60);
    const int min_particles = declare_parameter<int>("min_particles", 500);
    const int max_particles = declare_parameter<int>("max_particles", 2000);
    const int resample_interval = declare_parameter<int>("resample_interval", 1);
    const double pf_err = declare_parameter<double>("pf_err", 0.05);
    const double pf_z = declare_parameter<double>("pf_z", 2.33);
    const double recovery_alpha_fast = declare_parameter<double>("recovery_alpha_fast", 0.0);
    const double recovery_alpha_slow = declare_parameter<double>("recovery_alpha_slow", 0.0);

    if (global_frame_.empty() || odom_frame_.empty() || base_frame_.empty() ||
      map_topic_.empty() || scan_topic_.empty() || output_file_.empty() ||
      max_updates_ <= 0 || max_updates_ > 100 || max_sessions_ <= 0 ||
      max_sessions_ > 20 || max_beams <= 0 ||
      min_particles <= 0 || max_particles < min_particles || resample_interval <= 0 ||
      !std::isfinite(update_min_d_) || update_min_d_ < 0.0 ||
      !std::isfinite(update_min_a_) || update_min_a_ < 0.0)
    {
      throw std::invalid_argument("Invalid particle filter debug parameters");
    }
    resample_interval_ = static_cast<std::size_t>(resample_interval);

    loc::ParticleFilterOptions options;
    options.min_particles = static_cast<std::size_t>(min_particles);
    options.max_particles = static_cast<std::size_t>(max_particles);
    options.pf_err = pf_err;
    options.kld_normal_quantile = pf_z;
    options.recovery_alpha_fast = recovery_alpha_fast;
    options.recovery_alpha_slow = recovery_alpha_slow;
    auto motion_model = std::make_unique<loc::DifferentialMotionModel>(
      alpha1, alpha2, alpha3, alpha4, alpha5, loc::kDefaultRandomSeed);
    std::unique_ptr<loc::LaserModel> laser_model;
    if (model_type == "likelihood_field") {
      laser_model = std::make_unique<loc::LikelihoodFieldModel>(
        z_hit, z_rand, sigma_hit, static_cast<std::size_t>(max_beams));
    } else if (model_type == "beam") {
      laser_model = std::make_unique<loc::BeamModel>(
        z_hit, z_short, z_max, z_rand, sigma_hit, lambda_short,
        static_cast<std::size_t>(max_beams));
    } else {
      throw std::invalid_argument("Unsupported laser_model_type: " + model_type);
    }
    filter_ = std::make_unique<loc::ParticleFilter>(
      std::move(motion_model), std::move(laser_model), options, loc::kDefaultRandomSeed);

    const auto template_path = std::filesystem::path(
      ament_index_cpp::get_package_share_directory("mini_nav_pf_debug")) / "web" / "viewer.html";
    std::ifstream input(template_path);
    if (!input) {
      throw std::runtime_error("Cannot open particle filter HTML template");
    }
    html_template_.assign(
      std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>());
    if (html_template_.find(kDataPlaceholder) == std::string::npos) {
      throw std::runtime_error("Particle filter HTML template has no data placeholder");
    }

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    map_subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
      map_topic_, rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local(),
      std::bind(&PfDebugNode::OnMap, this, std::placeholders::_1));
    initial_pose_subscription_ = create_subscription<
      geometry_msgs::msg::PoseWithCovarianceStamped>(
      "initialpose", rclcpp::SystemDefaultsQoS(),
      std::bind(&PfDebugNode::OnInitialPose, this, std::placeholders::_1));
    scan_subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_, rclcpp::SensorDataQoS(),
      std::bind(&PfDebugNode::OnScan, this, std::placeholders::_1));
    RCLCPP_INFO(get_logger(), "Waiting for %s and a new /initialpose; HTML: %s",
      map_topic_.c_str(), output_file_.c_str());
  }

private:
  static constexpr const char * kDataPlaceholder = "__MINI_NAV_DEBUG_DATA__";

  void OnMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr message)
  {
    if (first_map_only_ && map_) {
      return;
    }
    const auto width = message->info.width;
    const auto height = message->info.height;
    const double map_yaw = tf2::getYaw(message->info.origin.orientation);
    if (StripLeadingSlash(message->header.frame_id) != global_frame_ ||
      width == 0U || height == 0U || !std::isfinite(message->info.resolution) ||
      message->info.resolution <= 0.0F ||
      message->data.size() != static_cast<std::size_t>(width) * height ||
      !std::isfinite(map_yaw) || std::abs(map_yaw) > 1.0e-6)
    {
      RCLCPP_WARN(get_logger(), "Ignoring incompatible localization map");
      return;
    }
    try {
      mini_nav_core::Costmap2D costmap(
        width, height, message->info.resolution,
        message->info.origin.position.x, message->info.origin.position.y,
        loc::LocalizationMap::kUnknownCost);
      for (unsigned int y = 0; y < height; ++y) {
        for (unsigned int x = 0; x < width; ++x) {
          const auto occupancy = message->data[static_cast<std::size_t>(y) * width + x];
          const unsigned char cost = occupancy < 0 ?
            loc::LocalizationMap::kUnknownCost :
            (occupancy > 0 ? loc::LocalizationMap::kLethalObstacle : 0);
          costmap.SetCost(x, y, cost);
        }
      }
      map_ = std::make_unique<loc::LocalizationMap>(costmap, max_distance_);
      map_message_ = *message;
      have_initial_pose_ = false;
      have_odom_pose_ = false;
      updates_ = 0;
      resample_count_ = 0;
      WriteHtml();
      RCLCPP_INFO(get_logger(), "Map ready (%u x %u); set an initial pose in RViz", width, height);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot prepare debug map: %s", error.what());
    }
  }

  void OnInitialPose(
    const geometry_msgs::msg::PoseWithCovarianceStamped::ConstSharedPtr message)
  {
    if (!map_) {
      RCLCPP_WARN(get_logger(), "Set initial pose after the debug node receives /map");
      return;
    }
    if (StripLeadingSlash(message->header.frame_id) != global_frame_) {
      RCLCPP_WARN(get_logger(), "Ignoring initial pose outside the map frame");
      return;
    }
    loc::Pose2D pose{
      message->pose.pose.position.x,
      message->pose.pose.position.y,
      tf2::getYaw(message->pose.pose.orientation)};
    loc::GridCell cell;
    if (!map_->TryGetCellFromWorld(pose.x, pose.y, cell) || !map_->IsKnownFree(cell)) {
      RCLCPP_WARN(get_logger(), "Initial pose must be in known free map space");
      return;
    }
    loc::Covariance3 covariance;
    constexpr int indices[3][3] = {{0, 1, 5}, {6, 7, 11}, {30, 31, 35}};
    for (std::size_t row = 0; row < 3; ++row) {
      for (std::size_t column = 0; column < 3; ++column) {
        covariance.At(row, column) = message->pose.covariance[indices[row][column]];
      }
    }
    try {
      filter_->InitializeLocalized(pose, covariance);
      have_initial_pose_ = true;
      have_odom_pose_ = false;
      updates_ = 0;
      resample_count_ = 0;
      sessions_.push_back(Session{
        next_session_id_++, rclcpp::Time(message->header.stamp).seconds(),
        pose, map_message_, {}});
      if (sessions_.size() > static_cast<std::size_t>(max_sessions_)) {
        sessions_.erase(sessions_.begin());
      }
      Capture("initialize", 0, rclcpp::Time(message->header.stamp), filter_->Estimate());
      WriteHtml();
      RCLCPP_INFO(get_logger(), "Initial particle cloud captured (%zu particles): %s",
        filter_->GetParticles().size(), output_file_.c_str());
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Cannot initialize debug filter: %s", error.what());
    }
  }

  bool GetTransformPose(
    const std::string & target, const std::string & source,
    const rclcpp::Time & stamp, loc::Pose2D & pose) const
  {
    try {
      const auto transform = tf_buffer_->lookupTransform(
        target, source, stamp, rclcpp::Duration::from_seconds(0.1));
      pose.x = transform.transform.translation.x;
      pose.y = transform.transform.translation.y;
      pose.yaw = tf2::getYaw(transform.transform.rotation);
      return std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.yaw);
    } catch (const tf2::TransformException &) {
      return false;
    }
  }

  loc::LaserScanData ConvertScan(const sensor_msgs::msg::LaserScan & message) const
  {
    loc::LaserScanData scan;
    scan.ranges.assign(message.ranges.begin(), message.ranges.end());
    scan.angle_min = message.angle_min;
    scan.angle_increment = message.angle_increment;
    scan.range_min = message.range_min;
    scan.range_max = message.range_max;
    if (min_range_override_ >= 0.0) {
      scan.range_min = std::max(scan.range_min, min_range_override_);
    }
    if (max_range_override_ > 0.0) {
      scan.range_max = std::min(scan.range_max, max_range_override_);
    }
    return scan;
  }

  void OnScan(const sensor_msgs::msg::LaserScan::ConstSharedPtr message)
  {
    if (!have_initial_pose_ || updates_ >= static_cast<std::size_t>(max_updates_)) {
      return;
    }
    loc::Pose2D odom_pose;
    loc::Pose2D laser_pose;
    const auto stamp = rclcpp::Time(message->header.stamp);
    if (!GetTransformPose(odom_frame_, base_frame_, stamp, odom_pose) ||
      !GetTransformPose(base_frame_, StripLeadingSlash(message->header.frame_id), stamp, laser_pose))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for scan-time odom/base/laser transforms");
      return;
    }
    if (have_odom_pose_ &&
      std::hypot(odom_pose.x - last_odom_pose_.x, odom_pose.y - last_odom_pose_.y) < update_min_d_ &&
      std::abs(loc::AngularDistance(odom_pose.yaw, last_odom_pose_.yaw)) < update_min_a_)
    {
      return;
    }

    const auto update = updates_ + 1U;
    try {
      if (have_odom_pose_) {
        filter_->MotionUpdate(last_odom_pose_, odom_pose);
        Capture("motion", update, stamp);
      }
      filter_->SensorUpdate(ConvertScan(*message), *map_, laser_pose);
      Capture("score", update, stamp);
      if (!filter_->NormalizeWeights()) {
        Capture("invalid_weights", update, stamp);
        WriteHtml();
        RCLCPP_WARN(get_logger(), "Scan produced unusable particle weights");
        return;
      }
      Capture("normalize", update, stamp);
      ++resample_count_;
      if (resample_count_ % resample_interval_ == 0U) {
        loc::ResampleTrace trace;
        if (!filter_->Resample(*map_, &trace)) {
          RCLCPP_WARN(get_logger(), "Particle resampling failed");
          return;
        }
        Capture("resample", update, stamp, std::nullopt, &trace);
      } else {
        Capture("hold", update, stamp);
      }
      const auto estimate = filter_->Estimate();
      if (!estimate.valid) {
        RCLCPP_WARN(get_logger(), "Particle estimate is invalid");
        return;
      }
      Capture("estimate", update, stamp, estimate);
      last_odom_pose_ = odom_pose;
      have_odom_pose_ = true;
      updates_ = update;
      WriteHtml();
      RCLCPP_INFO(get_logger(), "Captured update %zu/%d: %zu particles, estimate (%.2f, %.2f)",
        updates_, max_updates_, filter_->GetParticles().size(), estimate.pose.x, estimate.pose.y);
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "Debug filter update failed: %s", error.what());
    }
  }

  void Capture(
    const std::string & stage, std::size_t update, const rclcpp::Time & stamp,
    std::optional<loc::PoseEstimate> estimate = std::nullopt,
    const loc::ResampleTrace * trace = nullptr)
  {
    sessions_.back().frames.push_back(Snapshot{
      stage, update, stamp.seconds(), filter_->GetParticles(), estimate,
      trace ? std::optional<loc::ResampleTrace>(*trace) : std::nullopt});
  }

  std::string Serialize() const
  {
    std::ostringstream out;
    out << std::setprecision(std::numeric_limits<double>::max_digits10);
    out << "{\"version\":2,\"sessions\":[";
    for (std::size_t session_index = 0; session_index < sessions_.size(); ++session_index) {
      const auto & session = sessions_[session_index];
      const auto & session_map = session.map;
      if (session_index != 0U) {out << ',';}
      out << "{\"id\":" << session.id << ",\"initial_stamp\":";
      WriteNumber(out, session.initial_stamp);
      out << ",\"initial\":[";
      WriteNumber(out, session.initial_pose.x); out << ',';
      WriteNumber(out, session.initial_pose.y); out << ',';
      WriteNumber(out, session.initial_pose.yaw);
      out << "],\"map\":{\"width\":" << session_map.info.width
          << ",\"height\":" << session_map.info.height << ",\"resolution\":";
      WriteNumber(out, session_map.info.resolution);
      out << ",\"origin\":[";
      WriteNumber(out, session_map.info.origin.position.x);
      out << ',';
      WriteNumber(out, session_map.info.origin.position.y);
      out << "],\"data\":[";
      for (std::size_t i = 0; i < session_map.data.size(); ++i) {
        if (i != 0U) {out << ',';}
        out << static_cast<int>(session_map.data[i]);
      }
      out << "]},\"max_updates\":" << max_updates_ << ",\"frames\":[";
      for (std::size_t i = 0; i < session.frames.size(); ++i) {
        const auto & frame = session.frames[i];
        if (i != 0U) {out << ',';}
        out << "{\"stage\":\"" << frame.stage << "\",\"update\":" << frame.update
            << ",\"stamp\":";
        WriteNumber(out, frame.stamp);
        out << ",\"particles\":[";
        double min_weight = std::numeric_limits<double>::infinity();
        double max_weight = 0.0;
        for (std::size_t j = 0; j < frame.particles.size(); ++j) {
          const auto & particle = frame.particles[j];
          if (j != 0U) {out << ',';}
          out << '[';
          WriteNumber(out, particle.pose.x); out << ',';
          WriteNumber(out, particle.pose.y); out << ',';
          WriteNumber(out, particle.pose.yaw); out << ',';
          WriteNumber(out, particle.weight); out << ']';
          if (std::isfinite(particle.weight) && particle.weight >= 0.0) {
            min_weight = std::min(min_weight, particle.weight);
            max_weight = std::max(max_weight, particle.weight);
          }
        }
        double scaled_sum = 0.0;
        double scaled_square_sum = 0.0;
        if (max_weight > 0.0) {
          for (const auto & particle : frame.particles) {
            if (std::isfinite(particle.weight) && particle.weight >= 0.0) {
              const double scaled = particle.weight / max_weight;
              scaled_sum += scaled;
              scaled_square_sum += scaled * scaled;
            }
          }
        }
        out << "],\"ess\":";
        WriteNumber(out, scaled_square_sum > 0.0 ?
          scaled_sum * scaled_sum / scaled_square_sum : 0.0);
        out << ",\"weight_min\":";
        WriteNumber(out, std::isfinite(min_weight) ? min_weight : 0.0);
        out << ",\"weight_max\":";
        WriteNumber(out, max_weight);
        out << ",\"estimate\":";
        if (frame.estimate && frame.estimate->valid) {
          const auto & estimate = *frame.estimate;
          out << '[';
          WriteNumber(out, estimate.pose.x); out << ',';
          WriteNumber(out, estimate.pose.y); out << ',';
          WriteNumber(out, estimate.pose.yaw); out << ',';
          WriteNumber(out, std::sqrt(std::max(0.0, estimate.covariance.At(0, 0)))); out << ',';
          WriteNumber(out, std::sqrt(std::max(0.0, estimate.covariance.At(1, 1)))); out << ',';
          WriteNumber(out, std::sqrt(std::max(0.0, estimate.covariance.At(2, 2))));
          out << ']';
        } else {
          out << "null";
        }
        out << ",\"resample\":";
        if (frame.resample) {
          out << "{\"offset\":";
          WriteNumber(out, frame.resample->offset);
          out << ",\"weights\":[";
          for (std::size_t j = 0; j < frame.resample->source_weights.size(); ++j) {
            if (j != 0U) {out << ',';}
            WriteNumber(out, frame.resample->source_weights[j]);
          }
          out << "],\"draws\":[";
          for (std::size_t j = 0; j < frame.resample->draws.size(); ++j) {
            const auto & draw = frame.resample->draws[j];
            if (j != 0U) {out << ',';}
            out << '[';
            WriteNumber(out, draw.sample);
            out << ',';
            if (draw.recovery) {
              out << "null";
            } else {
              out << draw.source_index;
            }
            out << ']';
          }
          out << "]}";
        } else {
          out << "null";
        }
        out << '}';
      }
      out << "]}";
    }
    out << "]}";
    return out.str();
  }

  void WriteHtml() const
  {
    const auto path = std::filesystem::path(output_file_);
    if (!path.parent_path().empty()) {
      std::filesystem::create_directories(path.parent_path());
    }
    std::string html = html_template_;
    html.replace(html.find(kDataPlaceholder), std::char_traits<char>::length(kDataPlaceholder),
      Serialize());
    const auto temporary_path = path.string() + ".tmp";
    std::ofstream output(temporary_path, std::ios::binary | std::ios::trunc);
    if (!output) {
      throw std::runtime_error("Cannot write particle filter HTML output");
    }
    output << html;
    output.close();
    if (!output) {
      throw std::runtime_error("Particle filter HTML output write failed");
    }
    std::filesystem::rename(temporary_path, path);
  }

  std::string global_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string map_topic_;
  std::string scan_topic_;
  std::string output_file_;
  std::string html_template_;
  bool first_map_only_{true};
  bool have_initial_pose_{false};
  bool have_odom_pose_{false};
  double update_min_d_{0.25};
  double update_min_a_{0.2};
  double max_distance_{2.0};
  double max_range_override_{100.0};
  double min_range_override_{-1.0};
  int max_updates_{15};
  int max_sessions_{5};
  std::size_t next_session_id_{1};
  std::size_t updates_{0};
  std::size_t resample_count_{0};
  std::size_t resample_interval_{1};
  loc::Pose2D last_odom_pose_;
  nav_msgs::msg::OccupancyGrid map_message_;
  std::unique_ptr<loc::LocalizationMap> map_;
  std::unique_ptr<loc::ParticleFilter> filter_;
  std::vector<Session> sessions_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    initial_pose_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscription_;
};

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<PfDebugNode>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("pf_debug"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
