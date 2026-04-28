#include "hik_camera_ui/ros_worker.hpp"

RosWorker::RosWorker(QObject* parent) :QThread(parent),running_(false){
    node_ = rclcpp::Node::make_shared("hik_camera_ui_node");

    sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/image_raw",10,std::bind(&RosWorker::imageCallback,this,std::placeholders::_1)
    );
    param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
    node_, "hik_camera"  // 改成你相机驱动节点的实际名字
    );
}

RosWorker::~RosWorker(){
    stop();
    wait();//等待线程结束
}

void RosWorker::stop(){
    running_=false;
}

void RosWorker::run(){
    running_ = true;
    while (running_&&rclcpp::ok()){
        rclcpp::spin_some(node_);
        msleep(1);
    }
}

void RosWorker::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg){
    QImage image(
        msg->data.data(),
        msg->width,
        msg->height,
        msg->step,
        QImage::Format_BGR888
    );

    //必须copy，否则msg释放后图像数据失效
    emit imageReceived(image.copy());
}

void RosWorker::setParameter(const std::string& name, double value)
{
    if (!param_client_->service_is_ready()) {
        return;
    }
    param_client_->set_parameters({rclcpp::Parameter(name, value)});
}