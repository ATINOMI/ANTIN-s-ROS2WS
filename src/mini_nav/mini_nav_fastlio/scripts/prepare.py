#!/usr/bin/env python3
"""Prepare the pinned SCURM sources and the existing map assets."""
import hashlib
import json
import argparse
from pathlib import Path
import shutil
import subprocess

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT.parents[2]
UPSTREAM = ROOT / 'upstream/SCURM_SentryNavigation'
COMMIT = '46e6425c692ec98f8e65446fb6fdd360f44ef8e5'


def replace(path, old, new):
    text = path.read_text()
    if new in text:
        return
    if old not in text:
        raise RuntimeError(f'Patch context missing in {path}: {old[:80]}')
    path.write_text(text.replace(old, new))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mapping-only', action='store_true',
                        help='Prepare FAST-LIO2 without prior-map/ICP requirements')
    parser.add_argument('--localization-only', action='store_true',
                        help='Prepare FAST-LIO2 and ICP without the legacy map bundle')
    args = parser.parse_args()
    if not UPSTREAM.exists():
        UPSTREAM.parent.mkdir(parents=True, exist_ok=True)
        subprocess.run(['git', 'clone', 'https://github.com/PolarisXQ/SCURM_SentryNavigation.git',
                        str(UPSTREAM)], check=True)
        subprocess.run(['git', '-C', str(UPSTREAM), 'checkout', COMMIT], check=True)
    head = subprocess.check_output(['git', '-C', str(UPSTREAM), 'rev-parse', 'HEAD'], text=True).strip()
    if head != COMMIT:
        raise RuntimeError(f'Unexpected SCURM version: {head}')

    lio = UPSTREAM / 'FAST_LIO'
    cmake = lio / 'CMakeLists.txt'
    text = cmake.read_text().replace('-std=c++14', '-std=c++17').replace('-std=c++0x', '-std=c++17')
    text = text.replace('CMAKE_CXX_STANDARD 14', 'CMAKE_CXX_STANDARD 17')
    if 'find_package(tf2_ros REQUIRED)' not in text:
        text = text.replace('find_package(rclcpp REQUIRED)',
                            'find_package(rclcpp REQUIRED)\nfind_package(tf2_ros REQUIRED)\nfind_package(tf2_geometry_msgs REQUIRED)')
        text = text.replace('set(dependencies\n', 'set(dependencies\n  tf2_ros\n  tf2_geometry_msgs\n')
    cmake.write_text(text)
    replace(cmake, 'find_package(rclcpp REQUIRED)',
            'find_package(rclcpp REQUIRED)\nfind_package(diagnostic_msgs REQUIRED)')
    replace(cmake, 'set(dependencies\n', 'set(dependencies\n  diagnostic_msgs\n')
    replace(lio / 'package.xml', '<depend>rclcpp</depend>',
            '<depend>rclcpp</depend>\n  <depend>diagnostic_msgs</depend>')
    replace(lio / 'src/preprocess.h', '  MID360\n', '  MID360,\n  GAZEBO_SNAPSHOT = 8\n')
    replace(lio / 'src/preprocess.cpp',
            '    if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))',
            '    if (std::isfinite(added_pt.x) && std::isfinite(added_pt.y) && std::isfinite(added_pt.z) &&\n'
            '        added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind))')
    imu = lio / 'src/IMU_Processing.hpp'
    replace(imu, '  double first_lidar_time;', '  double first_lidar_time;\n  bool snapshot_input = false;')
    replace(imu, '  dt = note * (pcl_end_time - imu_end_time);', '  dt = pcl_end_time - imu_end_time;')
    replace(imu, '  /*** undistort each lidar point (backward propagation) ***/',
            '  if (snapshot_input) return;\n\n  /*** undistort each lidar point (backward propagation) ***/')
    source = lio / 'src/laserMapping.cpp'
    replace(source, '#include <nav_msgs/msg/odometry.hpp>',
            '#include <nav_msgs/msg/odometry.hpp>\n#include <diagnostic_msgs/msg/diagnostic_array.hpp>')
    replace(source, '    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;',
            '    rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr pubMatchQuality_;\n'
            '    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped_;')
    replace(source, '        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);',
            '        pubMatchQuality_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/scurm/match_quality", 10);\n'
            '        pubPath_ = this->create_publisher<nav_msgs::msg::Path>("/path", 20);')
    replace(source, '            /******* Publish odometry *******/',
            '            diagnostic_msgs::msg::DiagnosticArray match;\n'
            '            match.header.stamp = get_ros_time(lidar_end_time);\n'
            '            diagnostic_msgs::msg::DiagnosticStatus status;\n'
            '            status.name = "fastlio_prior_match";\n'
            '            status.hardware_id = "scurm_fastlio2";\n'
            '            const auto add_metric = [&](const std::string & key, double value) {\n'
            '                diagnostic_msgs::msg::KeyValue metric;\n'
            '                metric.key = key; metric.value = std::to_string(value);\n'
            '                status.values.push_back(metric);\n'
            '            };\n'
            '            add_metric("effective_points", effct_feat_num);\n'
            '            add_metric("matched_ratio", static_cast<double>(effct_feat_num) / std::max(1, feats_down_size));\n'
            '            add_metric("mean_abs_residual", effct_feat_num > 0 ? res_mean_last : std::numeric_limits<double>::infinity());\n'
            '            status.level = effct_feat_num >= 30 && res_mean_last <= 0.15 ? 0 : 2;\n'
            '            match.status.push_back(status);\n'
            '            pubMatchQuality_->publish(match);\n\n'
            '            /******* Publish odometry *******/')
    replace(source, '        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);',
            '        p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);\n'
            '        p_imu->snapshot_input = p_pre->lidar_type == GAZEBO_SNAPSHOT;')
    replace(source, 'this->declare_parameter<bool>("common.send_odom_base_tf", false);',
            'this->declare_parameter<bool>("common.send_odom_base_tf", false);\n'
            '        this->declare_parameter<bool>("publish_tf", true);')
    replace(source, '"/icp_result", 10, std::bind',
            '"/icp_result", rclcpp::QoS(1).transient_local(), std::bind')
    replace(source, 'imu_topic, 10, std::bind', 'imu_topic, rclcpp::SensorDataQoS(), std::bind')
    source.write_text(source.read_text().replace(
        '        pubOdomAftMapped->publish(odomAftMapped);\n        auto P = kf.get_P();',
        '        auto P = kf.get_P();'))
    replace(source, '            pubPath->publish(path);',
            '            path.header.stamp = msg_body_pose.header.stamp;\n'
            '            pubPath->publish(path);')
    replace(source, '        tf_broadcaster_->sendTransform(trans);',
            '        const V3D body_velocity = state_point.rot.conjugate() * state_point.vel;\n'
            '        odomAftMapped.twist.twist.linear.x = body_velocity.x();\n'
            '        odomAftMapped.twist.twist.linear.y = body_velocity.y();\n'
            '        odomAftMapped.twist.twist.linear.z = body_velocity.z();\n'
            '        if (!Measures.imu.empty()) {\n'
            '            const auto & gyro = Measures.imu.back()->angular_velocity;\n'
            '            odomAftMapped.twist.twist.angular.x = gyro.x - state_point.bg.x();\n'
            '            odomAftMapped.twist.twist.angular.y = gyro.y - state_point.bg.y();\n'
            '            odomAftMapped.twist.twist.angular.z = gyro.z - state_point.bg.z();\n'
            '        }\n'
            '        pubOdomAftMapped->publish(odomAftMapped);\n'
            '        if (this->get_parameter("publish_tf").as_bool()) tf_broadcaster_->sendTransform(trans);')
    source.write_text(source.read_text().replace(
        'if (effct_feat_num < 30 || res_mean_last > 0.15)',
        'if (locate_in_prior_map && (effct_feat_num < 30 || res_mean_last > 0.15))'))
    replace(source, '            publish_odometry(pubOdomAftMapped_);',
            '            if (locate_in_prior_map && (effct_feat_num < 30 || res_mean_last > 0.15)) {\n'
            '                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,\n'
            '                    "Withholding odometry: prior-map match is invalid");\n'
            '                return;\n'
            '            }\n'
            '            publish_odometry(pubOdomAftMapped_);')

    if args.mapping_only:
        subprocess.run(['git', '-C', str(UPSTREAM), 'diff', '--binary', '--output',
                        str(ROOT / 'patches/jazzy_sim.patch')], check=True)
        print(f'SCURM {COMMIT}; FAST-LIO2 mapping sources prepared')
        return

    icp = UPSTREAM / 'icp_relocalization'
    text = (icp / 'CMakeLists.txt').read_text().replace('CMAKE_CXX_STANDARD 14', 'CMAKE_CXX_STANDARD 17')
    if 'add_executable(sac_ia_gicp' in text:
        start = text.index('add_executable(sac_ia_gicp')
        end = text.index('# Install launch files', start)
        text = text[:start] + text[end:]
    text = text.replace('TARGETS icp_node transform_publisher sac_ia_gicp', 'TARGETS icp_node transform_publisher')
    text = text.replace('find_package(octomap_ros REQUIRED)\n', '')
    (icp / 'CMakeLists.txt').write_text(text)
    replace(icp / 'CMakeLists.txt', 'find_package(pcl_ros REQUIRED)',
            'find_package(pcl_ros REQUIRED)\nfind_package(PCL REQUIRED COMPONENTS common io filters registration)')
    replace(icp / 'CMakeLists.txt', 'add_executable(icp_node src/icp_node.cpp)',
            'add_executable(icp_node src/icp_node.cpp)\ntarget_link_libraries(icp_node ${PCL_LIBRARIES})')
    source = icp / 'src/icp_node.cpp'
    replace(source, 'this->declare_parameter("pcl_type","livox");',
            'this->declare_parameter("pcl_type","livox");\n'
            '        this->declare_parameter<bool>("rotate_input_x_180", true);')
    replace(source, '"icp_result", 10);', '"icp_result", rclcpp::QoS(1).transient_local());')
    replace(source, '        pcl::transformPointCloud(*input_cloud, *input_cloud, rotation);',
            '        if (this->get_parameter("rotate_input_x_180").as_bool())\n'
            '            pcl::transformPointCloud(*input_cloud, *input_cloud, rotation);')
    replace(source, '        double fitness_score = icp.getFitnessScore();',
            '        double fitness_score = icp.getFitnessScore(max_correspondence_distance);')
    replace(source, '        if (fitness_score < fitness_score_thre && icp.hasConverged())',
            '        if (std::isfinite(fitness_score) && fitness_score < fitness_score_thre && icp.hasConverged())')
    replace(source, '            converged_count++;\n            RCLCPP_INFO',
            '            converged_count++;\n            initGuess = icp.getFinalTransformation();\n            RCLCPP_INFO')

    if args.localization_only:
        subprocess.run(['git', '-C', str(UPSTREAM), 'diff', '--binary', '--output',
                        str(ROOT / 'patches/jazzy_sim.patch')], check=True)
        print(f'SCURM {COMMIT}; FAST-LIO2 prior localization sources prepared')
        return

    maps = ROOT / 'scurm_sim/maps'
    maps.mkdir(parents=True, exist_ok=True)
    bundle = WS / 'maps/fastlivo_static/map_static_2a3a449dc6cf48d3'
    manifest = json.loads((bundle / 'manifest.json').read_text())
    for filename, expected in manifest['files'].items():
        if hashlib.sha256((bundle / filename).read_bytes()).hexdigest() != expected:
            raise RuntimeError(f'Prior map bundle checksum mismatch: {filename}')
    alignment = json.loads((bundle / 'alignment.json').read_text())
    info = alignment['static_navigation_map']
    for kind in ['yaml', 'image']:
        filename = Path(info[f'source_{kind}'])
        if hashlib.sha256(filename.read_bytes()).hexdigest() != info[f'source_{kind}_sha256']:
            raise RuntimeError(f'Static map changed: {filename}')
    shutil.copyfile(bundle / 'geometry.pcd', maps / 'prior.pcd')
    (maps / 'provenance.json').write_text(json.dumps({
        'scurm_commit': COMMIT, 'bundle': str(bundle), 'files': manifest['files'],
        'static_navigation_map': info}, indent=2) + '\n')
    subprocess.run(['git', '-C', str(UPSTREAM), 'diff', '--binary', '--output',
                    str(ROOT / 'patches/jazzy_sim.patch')], check=True)
    print(f'SCURM {COMMIT}; verified prior and original 2D map')


if __name__ == '__main__':
    main()
