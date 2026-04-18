#include "../include/hik_camera/hik_camera_node.hpp"

HikCameraNode::HikCameraNode() : Node("hik_camera")
{

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