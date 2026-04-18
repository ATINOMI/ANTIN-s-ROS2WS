#include "../include/apple_detector/apple_detector_node.hpp"

AppleDetectorNode::AppleDetectorNode() : Node("apple_detector")
{
    this->declare_parameter<std::string>("image_path", "/home/a/ros2_ws/src/apple_detector/picture/apple2.jpg");
    publisher_ = image_transport::create_publisher(this, "/apple_detector/result");
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AppleDetectorNode::detect_and_publish, this)
    );
}

cv::Mat AppleDetectorNode::detectApple(cv::Mat& img)
{
    if (img.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "图片加载失败: %s", image_path_.c_str());
        return cv::Mat();
    }

    // 第一步：颜色分离
    // BGR → HSV，HSV 更适合颜色阈值分割
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // 红色在 HSV 中跨越 0°，需要两个区间，同时由于苹果是红色加黄色，所以扩宽阈值
    cv::Mat apple1, apple2, apple;
    cv::inRange(hsv, cv::Scalar(0, 60, 60),   cv::Scalar(30, 255, 255),  apple1);  // 红黄色低区间
    cv::inRange(hsv, cv::Scalar(155, 60, 60), cv::Scalar(180, 255, 255), apple2);  // 红黄色高区间
    apple = apple1 | apple2;

    // 第二步：形态学处理
    cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));  // 闭运算：填补苹果内部孔洞
    cv::Mat open_kernel  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));   // 开运算：去除小噪点

    cv::Mat apple_cleaned;
    cv::morphologyEx(apple,         apple_cleaned, cv::MORPH_CLOSE, close_kernel);
    cv::morphologyEx(apple_cleaned, apple_cleaned, cv::MORPH_OPEN,  open_kernel);

    cv::namedWindow("apple_mask",cv::WINDOW_NORMAL);
    cv::imshow("apple_mask", apple_cleaned);

    // 第三步：轮廓检测
    cv::findContours(apple_cleaned, apple_contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    RCLCPP_INFO(this->get_logger(), "原始轮廓数量：%zu", apple_contours_.size());

    cv::Mat display = img.clone();

    // 第四步：筛选 + 可视化
    int idx = 0;

    // 第一次遍历找最大面积
    double max_area = 0;
    for (auto& contour : apple_contours_)
        max_area = std::max(max_area, cv::contourArea(contour));

    // 第二次遍历筛选
    for (auto& contour : apple_contours_)
    {
        double area = cv::contourArea(contour);
        if (area < max_area * 0.1)  // 过滤小于最大面积10%的噪点
            continue;

        double perimeter   = cv::arcLength(contour, true);
        double circularity = 4 * M_PI * area / (perimeter * perimeter);  // 圆度公式：4πA / P²
        if (circularity < 0.3)
            continue;
        int thickness = std::max(1, img.rows / 500);
        int dot_radius = std::max(4, img.rows / 200);


        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);
        cv::circle(display, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), thickness);
        cv::circle(display, center, dot_radius, cv::Scalar(0, 0, 255), -1);

        RCLCPP_INFO(this->get_logger(), "[%d] 面积: %.1f 圆度: %.2f", idx, area, circularity);
        idx++;
    }

    RCLCPP_INFO(this->get_logger(), "筛选后轮廓数量：%d", idx);
    return display;
}

void AppleDetectorNode::detect_and_publish()
{
    image_path_ = this->get_parameter("image_path").as_string();
    cv::Mat img = cv::imread(image_path_);

    cv::Mat result = detectApple(img);
    if (result.empty())
        return;

    RCLCPP_INFO(this->get_logger(), "result size: %dx%d", result.cols, result.rows);

    cv::namedWindow("apple_detection_result",cv::WINDOW_NORMAL);    
    cv::imshow("apple_detection_result", result);
    cv::waitKey(100);

    cv::imwrite("/home/a/ros2_ws/src/apple_detector/picture/result_apple.jpg", result);
    RCLCPP_INFO(this->get_logger(), "结果已保存");

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg();
    publisher_.publish(*msg);
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AppleDetectorNode>();
    
    std::thread spin_thread([&node]() {
        rclcpp::spin(node);
    });
    
    spin_thread.join();  // 等spin结束
    rclcpp::shutdown();
    return 0;
}