#pragma once

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <thread>
#include <atomic>
#include <MvCameraControl.h>
#include <opencv2/opencv.hpp>

class HikCameraNode : public rclcpp::Node
{
    public:
        HikCameraNode();
        ~HikCameraNode();
    
    private:
        void grab_thread();
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_ ;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
        //取帧线程
        std::thread grab_thread_;
        std::atomic<bool> running_;

        //海康相机句柄
        void* handle_;

        // camera_info 管理器
        std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;



};
