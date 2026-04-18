#include "../include/hik_camera/hik_camera_node.hpp"

HikCameraNode::HikCameraNode() : Node("hik_camera"),handle_(nullptr),running_(false)
{
    image_pub_ =this->create_publisher<sensor_msgs::msg::Image>("image_raw",10);
    camera_info_pub_ =this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info",10);
    //初始化相机信息管理器
    camera_info_manager_=std::make_shared<camera_info_manager::CameraInfoManager>(this);

}

HikCameraNode::~HikCameraNode()
{

}

void HikCameraNode::grab_thread()
{

}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}