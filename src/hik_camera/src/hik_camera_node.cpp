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
    //注意：工业相机的bayerRG8不是真正的彩色格式，只是传感器数据，需要OpenCV进行去马赛克化
    MV_CC_SetEnumValueByString(handle_,"PixelFormat","BayerRG8");
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

    //声明参数
    this->declare_parameter("exposure_time",5000.0);
    this->declare_parameter("gain",10.0);
    this->declare_parameter("frame_rate",90.0);

    //应用初始参数到相机
    MV_CC_SetFloatValue(handle_,"ExposureTime",5000.0);
    MV_CC_SetFloatValue(handle_,"Gain",10.0);
    MV_CC_SetEnumValueByString(handle_, "ExposureAuto", "Off");
    MV_CC_SetEnumValueByString(handle_, "GainAuto", "Off");



    //注册参数变化回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params){
            for (const auto & param : params){
                if (param.get_name()=="exposure_time"){
                    MV_CC_SetFloatValue(handle_,"ExposureTime",param.as_double());
                }
                else if (param.get_name()=="gain"){
                    MV_CC_SetFloatValue(handle_,"Gain",param.as_double());
                }
                else if (param.get_name()=="frame_rate")
                {
                    MV_CC_SetBoolValue( handle_,"AcquisitionFrameRateEnable",true);
                    MV_CC_SetFloatValue(handle_,"AcquisitionFrameRate",param.as_double());
                }
                
            }
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            return result;
        }
    );

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
        //把海康buffer包装成OpenCV矩阵，不拷贝数据，只是借用指针
        cv::Mat bayer(frame.stFrameInfo.nExtendHeight,frame.stFrameInfo.nExtendWidth,CV_8UC1,frame.pBufAddr);
        //Bayer解码:把棋盘格排列的单通道数据还原成真正的BGR彩色图
        cv::Mat bgr;
        cv::cvtColor(bayer,bgr,cv::COLOR_BayerBG2BGR);//BayerRGBGR是插值算法，根据周围RGB邻居推算完整的BGR彩色图，这个过程加去马赛克

        img_msg->encoding = "bgr8";
        img_msg->step     = frame.stFrameInfo.nExtendWidth*3;//BGR三个通道，字节数是mono8的3倍
        img_msg->data.assign(bgr.data,bgr.data + bgr.total()*3);

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