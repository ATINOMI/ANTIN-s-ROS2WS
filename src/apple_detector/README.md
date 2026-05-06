# 苹果识别节点 —— 实现思路与总结

## 整体实现思路

基于传统CV pipeline，不使用深度学习，完全依赖颜色特征和形状特征完成苹果检测：

```
BGR → HSV 颜色分割 → 形态学处理 → 轮廓检测 → 面积筛选 → 圆形度筛选 → 绘制外接圆与圆心
```

---

## ROS2 节点架构

### 节点结构

代码拆分为 `.hpp` + `.cpp` 两个文件，头文件只存放类声明，源文件存放具体实现。

```
apple_detector/
├── include/apple_detector/
│   └── apple_detector_node.hpp   # 类声明、成员变量、函数原型
└── src/
    └── apple_detector_node.cpp   # 构造函数、detectApple、detect_and_publish、main
```

### 类设计

```cpp
class AppleDetectorNode : public rclcpp::Node
{
public:
    AppleDetectorNode();

private:
    void detect_and_publish();   // 定时器回调：读图、调用检测、发布结果
    cv::Mat detectApple(cv::Mat& img);  // 核心检测函数，返回处理后图像

    std::string image_path_;
    std::vector<std::vector<cv::Point>> apple_contours_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};
```

成员变量统一加 `_` 后缀，与局部变量区分。

### 构造函数

声明图片路径参数，创建图像发布者和定时器，每秒触发一次检测：

```cpp
AppleDetectorNode::AppleDetectorNode() : Node("apple_detector")
{
    this->declare_parameter<std::string>("image_path", "/home/a/ros2_ws/src/apple_detector/picture/apple.jpg");
    publisher_ = image_transport::create_publisher(this, "/apple_detector/result");
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&AppleDetectorNode::detect_and_publish, this)
    );
}
```

### detect_and_publish

定时器回调函数，负责读图、调用检测、显示、保存、发布：

```cpp
void AppleDetectorNode::detect_and_publish()
{
    image_path_ = this->get_parameter("image_path").as_string();
    cv::Mat img = cv::imread(image_path_);

    cv::Mat result = detectApple(img);
    if (result.empty())
        return;

    cv::namedWindow("apple_detection_result", cv::WINDOW_NORMAL);
    cv::imshow("apple_detection_result", result);
    cv::waitKey(30);

    cv::imwrite("/home/a/ros2_ws/src/apple_detector/picture/result_apple.jpg", result);

    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", result).toImageMsg();
    publisher_.publish(*msg);
}
```

### main 函数

将 `rclcpp::spin` 放到子线程，主线程不被占用，OpenCV窗口响应流畅：

```cpp
int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AppleDetectorNode>();
    std::thread spin_thread([&node]() { rclcpp::spin(node); });
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
}
```

---

## 各步骤详解

### 第一步：HSV 颜色分割

苹果为红黄混色，红色在HSV中跨越0°需要两个区间，同时黄色区域H值在15~30之间，需要扩宽阈值覆盖。

```cpp
cv::Mat hsv;
cv::cvtColor(img, hsv, cv::COLOR_BGR2HSV);

cv::Mat apple1, apple2, apple;
cv::inRange(hsv, cv::Scalar(0, 60, 60),   cv::Scalar(30, 255, 255),  apple1);  // 红黄色低区间
cv::inRange(hsv, cv::Scalar(155, 60, 60), cv::Scalar(180, 255, 255), apple2);  // 红色高区间
apple = apple1 | apple2;
```

### 第二步：形态学处理

闭运算填补颜色分割后苹果内部的孔洞，开运算去除小噪点。

```cpp
cv::Mat close_kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25, 25));
cv::Mat open_kernel  = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

cv::Mat apple_cleaned;
cv::morphologyEx(apple,         apple_cleaned, cv::MORPH_CLOSE, close_kernel);
cv::morphologyEx(apple_cleaned, apple_cleaned, cv::MORPH_OPEN,  open_kernel);
```

### 第三步：轮廓检测

只检测最外层轮廓，避免内部孔洞产生嵌套轮廓。

```cpp
cv::findContours(apple_cleaned, apple_contours_, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
```

### 第四步：面积筛选

固定面积阈值对不同分辨率图片失效，改为相对最大轮廓面积的比例筛选，过滤远小于主目标的噪点轮廓。

```cpp
double max_area = 0;
for (auto& contour : apple_contours_)
    max_area = std::max(max_area, cv::contourArea(contour));

for (auto& contour : apple_contours_)
{
    double area = cv::contourArea(contour);
    if (area < max_area * 0.1)
        continue;
    // ...
}
```

### 第五步：圆形度筛选

苹果为近圆形目标，用圆形度公式筛选非圆形轮廓：

$$C = \frac{4\pi \cdot Area}{Perimeter^2}$$

完美圆 $C = 1.0$，苹果实测约 $0.4$ 左右，阈值设为 $0.3$。

```cpp
double perimeter   = cv::arcLength(contour, true);
double circularity = 4 * M_PI * area / (perimeter * perimeter);
if (circularity < 0.3)
    continue;
```

### 第六步：绘制外接圆与圆心

线宽根据图片分辨率自适应，避免高分辨率图片下线条过细。

```cpp
int thickness  = std::max(1, img.rows / 500);
int dot_radius = std::max(4, img.rows / 200);

cv::Point2f center;
float radius;
cv::minEnclosingCircle(contour, center, radius);
cv::circle(display, center, static_cast<int>(radius), cv::Scalar(0, 255, 0), thickness);
cv::circle(display, center, dot_radius, cv::Scalar(0, 0, 255), -1);
```

---

## 遇到的困难与解决方法

### 问题一：黄色区域被过滤，轮廓不完整

**现象**：mask窗口中苹果出现黑白条纹，圆形度只有0.42，低于初始阈值0.7导致检测失败。

**原因**：苹果为红黄混色，初始HSV阈值S=150、V=220过于严格，黄色区域饱和度和亮度较低被过滤掉。

**解决**：将S和V阈值放宽至60，同时将H的高端扩展至30覆盖黄色区域：
```cpp
// 修改前
cv::inRange(hsv, cv::Scalar(0, 150, 220), cv::Scalar(10, 255, 255), apple1);
// 修改后
cv::inRange(hsv, cv::Scalar(0, 60, 60), cv::Scalar(30, 255, 255), apple1);
```

### 问题二：固定面积阈值对不同分辨率失效

**现象**：换用高分辨率图片后，苹果面积达到9,413,518，而固定阈值 `area < 5000` 会把苹果本身也过滤掉。

**解决**：改为相对最大轮廓面积的比例筛选，与分辨率无关：
```cpp
if (area < max_area * 0.1)
    continue;
```

### 问题三：高分辨率图片下绘制线条过细

**现象**：8192×6144的图片上，线宽2像素几乎不可见。

**解决**：根据图片行数动态计算线宽：
```cpp
int thickness  = std::max(1, img.rows / 500);
int dot_radius = std::max(4, img.rows / 200);
```
