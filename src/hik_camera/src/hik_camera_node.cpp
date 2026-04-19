#include "../include/hik_camera/hik_camera_node.hpp"

HikCameraNode::HikCameraNode() : Node("hik_camera"),running_(false),handle_(nullptr)
{
    image_pub_ =this->create_publisher<sensor_msgs::msg::Image>("image_raw",10);
    camera_info_pub_ =this->create_publisher<sensor_msgs::msg::CameraInfo>("camera_info",10);
    //初始化相机信息管理器
    camera_info_manager_=std::make_shared<camera_info_manager::CameraInfoManager>(this);

    MV_CC_Initialize();

    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list,0,sizeof(MV_CC_DEVICE_INFO_LIST));
    MV_CC_EnumDevices(MV_USB_DEVICE,&device_list);

    if (device_list.nDeviceNum==0){
        RCLCPP_ERROR(this->get_logger(),"No camera found!");
        return;
    }

    //传建句柄
    int nRet = MV_CC_CreateHandle(&handle_,device_list.pDeviceInfo[0]);
    if (nRet!=MV_OK){
        RCLCPP_ERROR(this->get_logger(),"Create handle failed!");
        return;
    }

    //打开设备
    nRet = MV_CC_OpenDevice(handle_);
    if (nRet !=MV_OK){
        RCLCPP_ERROR(this->get_logger(),"Open camera failed!");
        return;       
    }

    //关闭触发模式，开启连续模式
    nRet = MV_CC_SetEnumValue(handle_,"TriggerMode",0);
    if (nRet != MV_OK){
        RCLCPP_ERROR(this->get_logger(),"Set triggermode failed!");
        return;
    }

    //开始取流
    nRet = MV_CC_StartGrabbing(handle_);
    if (nRet != MV_OK){
        RCLCPP_ERROR(this->get_logger(),"Start grabbing failed!");
        return;        
    }

    //启动取帧线程
    running_ = true;
    grab_thread_ = std::thread(&HikCameraNode::grab_thread,this);
}

HikCameraNode::~HikCameraNode()
{
    running_ = false;
    if (grab_thread_.joinable()){
        grab_thread_.join();
    }
    //停止取帧
    MV_CC_StopGrabbing(handle_);
    
    //关闭设备
    MV_CC_CloseDevice(handle_);

    //销毁句柄
    MV_CC_DestroyHandle(handle_);

    //结束
    MV_CC_Finalize();
}

void HikCameraNode::grab_thread()
{
    MV_FRAME_OUT frame;
    memset(&frame, 0, sizeof(MV_FRAME_OUT));

    while (running_){
        int nRet = MV_CC_GetImageBuffer(handle_, &frame, 1000);
        if (nRet!=MV_OK){
            RCLCPP_WARN(this->get_logger(),"Get image buffer failed!");
            continue;
        }

        //TODO:转成ROS2消息并发布

        auto img_msg = std::make_shared<sensor_msgs::msg::Image>();
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = "camera";
        img_msg->width = frame.stFrameInfo.nExtendWidth;
        img_msg->height = frame.stFrameInfo.nExtendHeight;
        img_msg->encoding = "mono8";
        img_msg->step = frame.stFrameInfo.nExtendWidth*1;
        img_msg->data.assign(frame.pBufAddr,frame.pBufAddr+frame.stFrameInfo.nFrameLen);

        image_pub_->publish(*img_msg);

        MV_CC_FreeImageBuffer(handle_, &frame);
    }
}

int main(int argc,char** argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<HikCameraNode>());
    rclcpp::shutdown();
    return 0;
}