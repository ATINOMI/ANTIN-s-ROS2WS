#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <image_transport/image_transport.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;


class ArmorDetectorNode : public rclcpp::Node
{
public:
    ArmorDetectorNode() : Node("armor_detector")
    {
        this->declare_parameter<std::string>("image_path", "/home/a/rm_try/armor2.jpeg");

        publisher_ = image_transport::create_publisher(this, "/armor_detector/result");

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&ArmorDetectorNode::detect_and_publish, this)
        );
    }

private:
    void detect_and_publish()
    {
        std::string image_path = this->get_parameter("image_path").as_string();
        Mat img = imread(image_path);

        if (img.empty())
        {
            RCLCPP_ERROR(this->get_logger(), "picture loading failed: %s", image_path.c_str());
            return;
        }

        // 第一步：颜色分离
        Mat hsv;
        cvtColor(img, hsv, COLOR_BGR2HSV);

        Mat blue_mask;
        inRange(hsv, Scalar(100, 150, 215), Scalar(130, 255, 255), blue_mask);

        Mat red_mask1, red_mask2, red_mask;
        inRange(hsv, Scalar(0, 150, 220), Scalar(10, 255, 255), red_mask1);
        inRange(hsv, Scalar(170, 150, 220), Scalar(180, 255, 255), red_mask2);
        red_mask = red_mask1 | red_mask2;

        // 第二步：形态学处理
        Mat close_kernel = getStructuringElement(MORPH_RECT, Size(15, 15));
        Mat open_kernel  = getStructuringElement(MORPH_RECT, Size(3, 3));

        Mat blue_cleaned, red_cleaned;
        morphologyEx(blue_mask,    blue_cleaned, MORPH_CLOSE, close_kernel);
        morphologyEx(blue_cleaned, blue_cleaned, MORPH_OPEN,  open_kernel);
        morphologyEx(red_mask,     red_cleaned,  MORPH_CLOSE, close_kernel);
        morphologyEx(red_cleaned,  red_cleaned,  MORPH_OPEN,  open_kernel);

        // 第三步：轮廓检测
        std::vector<std::vector<Point>> blue_contours;
        findContours(blue_cleaned, blue_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
        RCLCPP_INFO(this->get_logger(), "原始轮廓数量： %zu", blue_contours.size());

        Mat display = img.clone();
        drawContours(display, blue_contours, -1, Scalar(0, 255, 0), 2);

        // 第四步：灯条筛选
        std::vector<RotatedRect> light_bars;
        srand(42);
        int idx = 0;

        for (auto& contour : blue_contours)
        {
            float area = contourArea(contour);
            if (area < 100 || area > 5000)
                continue;

            RotatedRect rect = minAreaRect(contour);

            float width      = rect.size.width;
            float height     = rect.size.height;
            float long_side  = max(width, height);
            float short_side = min(width, height);
            float ratio      = long_side / short_side;

            float angle = rect.angle;
            if (rect.size.width < rect.size.height)
                angle += 90;

            if (rect.size.width < rect.size.height)
                continue;
            if (rect.angle < 50)
                continue;

            Scalar color(0, 0, 255);

            RCLCPP_INFO(this->get_logger(), "[%d] 面积:%.1f 长宽比:%.2f 角度:%.1f", idx, area, ratio, angle);

            Point2f pts[4];
            rect.points(pts);
            for (int j = 0; j < 4; j++)
                line(display, pts[j], pts[(j+1)%4], color, 2);

            putText(display,
                    "[" + std::to_string(idx) + "]" + std::to_string(ratio).substr(0, 4),
                    rect.center,
                    FONT_HERSHEY_SIMPLEX,
                    0.5, color, 1);

            light_bars.push_back(rect);
            idx++;
        }

        RCLCPP_INFO(this->get_logger(), "筛选后灯条数量：%zu", light_bars.size());

        auto msg = cv_bridge::CvImage(
            std_msgs::msg::Header(), "bgr8", display
        ).toImageMsg();
        publisher_.publish(*msg);
    }

    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
