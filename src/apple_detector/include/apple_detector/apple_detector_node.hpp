#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.hpp>

class AppleDetectorNode:public rclcpp::Node
{
public:
    AppleDetectorNode();


private:
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    cv::Mat img;
    std::string image_path_;
    std::vector<std::vector<cv::Point>> apple_contours_;
    void detect_and_publish();
    cv::Mat detectApple(cv::Mat& img);

};
