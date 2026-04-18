#include "../include/apple_detector/apple_detector_node.hpp"

/**
 * @brief 构造函数
 * 
 * 功能：
 * 1. 初始化 ROS 2 节点名称
 * 2. 声明并设置参数 image_path（待检测图片路径）
 * 3. 创建 image_transport 发布者
 * 4. 创建定时器，周期性执行苹果检测
 */
AppleDetectorNode::AppleDetectorNode() : Node("apple_detector")
{
    // 声明参数：待检测图片路径（支持 ros2 param 动态修改）
    this->declare_parameter<std::string>(
        "image_path",
        "/home/a/ros2_ws/src/apple_detector/picture/apple.jpg"
    );

    // 创建图像发布者，发布检测结果图像
    publisher_ = image_transport::create_publisher(
        this,
        "/apple_detector/result"
    );

    // 创建定时器：每 1 秒执行一次 detect_and_publish
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AppleDetectorNode::detect_and_publish, this)
    );
}

/**
 * @brief 核心苹果检测函数
 * 
 * @param img 输入原始 BGR 图像
 * @return cv::Mat 画好检测结果的图像
 * 
 * 算法流程：
 * 1. BGR → HSV 颜色空间转换
 * 2. 红色区间阈值分割（HSV 双区间）
 * 3. 形态学闭运算 + 开运算去噪
 * 4. 轮廓检测
 * 5. 基于面积 + 圆度筛选苹果目标
 * 6. 在原图上绘制圆形检测结果
 */
cv::Mat AppleDetectorNode::detectApple(cv::Mat& img)
{
    // 图像合法性检查
    if (img.empty())
    {
        RCLCPP_ERROR(
            this->get_logger(),
            "图片加载失败: %s",
            image_path_.c_str()
        );
        return cv::Mat();
    }

    /* ======================= 第一步：颜色分离 ======================= */

    // BGR → HSV，HSV 更适合颜色阈值分割
    cv::Mat hsv;
    cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

    // 红色在 HSV 中跨越 0°，需要两个区间，同时由于苹果是红色加黄色，所以扩宽阈值
    cv::Mat apple1, apple2, apple;

    // 红黄色低区间
    cv::inRange(
        hsv,
        cv::Scalar(0, 60, 60),
        cv::Scalar(30, 255, 255),
        apple1
    );

    // 红黄色高区间
    cv::inRange(
        hsv,
        cv::Scalar(155, 60, 60),
        cv::Scalar(180, 255, 255),
        apple2
    );

    // 合并两个红色掩码
    apple = apple1 | apple2;

    /* ======================= 第二步：形态学处理 ======================= */

    // 闭运算：填补苹果内部孔洞
    cv::Mat close_kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));

    // 开运算：去除小噪点
    cv::Mat open_kernel =
        cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    cv::Mat apple_cleaned;

    // 先闭后开
    cv::morphologyEx(
        apple,
        apple_cleaned,
        cv::MORPH_CLOSE,
        close_kernel
    );

    cv::morphologyEx(
        apple_cleaned,
        apple_cleaned,
        cv::MORPH_OPEN,
        open_kernel
    );

    cv::imshow("apple_mask",apple_cleaned);

    /* ======================= 第三步：轮廓检测 ======================= */

    // 查找外部轮廓
    cv::findContours(
        apple_cleaned,
        apple_contours_,
        cv::RETR_EXTERNAL,
        cv::CHAIN_APPROX_SIMPLE
    );

    RCLCPP_INFO(
        this->get_logger(),
        "原始轮廓数量：%zu",
        apple_contours_.size()
    );

    // 用于显示检测结果的图像
    cv::Mat display = img.clone();

    /* ======================= 第四步：筛选 + 可视化 ======================= */

    int idx = 0;
    for (auto& contour : apple_contours_)
    {
        // 计算轮廓面积
        double area = cv::contourArea(contour);

        // 面积过滤：去掉极小噪声
        if (area < 100)
            continue;

        // 计算周长
        double perimeter = cv::arcLength(contour, true);

        // 圆度公式：4πA / P²
        double circularity =
            4 * M_PI * area / (perimeter * perimeter);

        // 圆度过滤：排除非近圆形目标
        if (circularity < 0.3)
            continue;

        // 最小外接圆
        cv::Point2f center;
        float radius;
        cv::minEnclosingCircle(contour, center, radius);

        // 画圆（绿色）
        cv::circle(
            display,
            center,
            static_cast<int>(radius),
            cv::Scalar(0, 255, 0),
            2
        );

        // 画圆心（红色）
        cv::circle(
            display,
            center,
            4,
            cv::Scalar(0, 0, 255),
            -1
        );

        RCLCPP_INFO(
            this->get_logger(),
            "[%d] 面积: %.1f 圆度: %.2f",
            idx,
            area,
            circularity
        );

        idx++;
        RCLCPP_INFO(
            this->get_logger(),
            "筛选后轮廓数量：%d",
            idx
        );
    }

    return display;
}

/**
 * @brief 定时器回调函数
 * 
 * 功能：
 * 1. 读取参数 image_path
 * 2. 加载图片
 * 3. 调用 detectApple
 * 4. 显示、保存并发布检测结果
 */
void AppleDetectorNode::detect_and_publish()
{
    // 从参数服务器读取图片路径
    image_path_ = this->get_parameter("image_path").as_string();

    // 读取图像
    cv::Mat img = cv::imread(image_path_);

    // 执行苹果检测
    cv::Mat result = detectApple(img);
    if (result.empty())
        return;

    RCLCPP_INFO(
        this->get_logger(),
        "result size: %dx%d",
        result.cols,
        result.rows
    );

    /* ======================= 本地显示 ======================= */

    cv::imshow("apple_detection_result", result);
    cv::waitKey(1);

    /* ======================= 保存结果 ======================= */

    cv::imwrite(
        "/home/a/ros2_ws/src/apple_detector/picture/result_apple.jpg",
        result
    );

    RCLCPP_INFO(this->get_logger(), "结果已保存");

    /* ======================= ROS 2 图像发布 ======================= */

    auto msg = cv_bridge::CvImage(
        std_msgs::msg::Header(),
        "bgr8",
        result
    ).toImageMsg();

    publisher_.publish(*msg);
}

/**
 * @brief 主函数
 * 
 * 功能：
 * 1. 初始化 ROS 2
 * 2. 创建并运行 AppleDetectorNode
 * 3. 进入 spin 循环
 */
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AppleDetectorNode>());
    rclcpp::shutdown();
    return 0;
}