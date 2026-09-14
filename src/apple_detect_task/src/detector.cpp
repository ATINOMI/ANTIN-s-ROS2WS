/**
  ******************************************************************************
  * @file      detector.cpp
  * @author    ANTINOMI
  * @date      2026-07-30
  * @brief     苹果目标检测算法的具体实现。
  *            本文件完成 HSV 红色分割、形态学处理、轮廓筛选、
  *            凸包计算以及最小外接圆圆心提取。
  ******************************************************************************
  * @attention
  *
  * 本文件属于 apple_detect_task ROS 2 功能包。
  * 检测结果中的坐标单位为像素，输入图像格式为 BGR。
  ******************************************************************************
  */

/* Includes --------------------------------------------------------------*/

#include "apple_detect_task/detector.hpp"

/**
 * @brief 对单帧 BGR 图像执行苹果检测。
 *
 * 检测流程包括 HSV 红色分割、两个红色区间合并、
 * 形态学膨胀、最大轮廓提取、凸包计算和最小外接圆计算。
 *
 * @param input 输入的 BGR 图像。
 * @param output 输出图像。函数会复制输入图像，并在其上绘制凸包和圆心。
 * @return 当前帧的检测结果。
 */

/* Member Function --------------------------------------------------------*/

DetectionResult Detector::detect(
  const cv::Mat& input,
  cv::Mat&       output) const
{

  //创建结构体，用于储存苹果检测的结果
  DetectionResult result;

  //检查输入的图像是否为空
  if (input.empty()) {
    output.release();
    return result;
  }

  //深拷贝函数，用于隔离输入与输出图像，防止共用内存造成的处理混乱风险
  output = input.clone();


  /*
  ====================================
                Part 1 
                hsv过滤
  ====================================
  */

  //用于存储hsv转化后的input图像
  cv::Mat frame_hsv;
  cv::cvtColor(input, frame_hsv, cv::COLOR_BGR2HSV);

  // 红色在 HSV 色彩空间中跨越 0°，使用两个区间进行提取。
  cv::Mat red_mask_low;
  cv::Mat red_mask_high;

  cv::inRange(
    frame_hsv,
    cv::Scalar(1, 90, 120),
    cv::Scalar(10, 255, 255),
    red_mask_low);

  cv::inRange(
    frame_hsv,
    cv::Scalar(170, 90, 120),
    cv::Scalar(179, 255, 255),
    red_mask_high);

  //合并两个区间
  cv::Mat frame_mask;
  cv::bitwise_or(red_mask_low, red_mask_high, frame_mask);



  /*
  ====================================
                Part 2 
              形态学处理
  ====================================
  */

  // 形态学膨胀：扩大目标区域，连接局部断裂。
  const cv::Mat kernel = cv::getStructuringElement(
    cv::MORPH_ELLIPSE,//椭圆核，用于边缘平滑化
    cv::Size(10, 10));

  //苹果存在花纹与斑点，hsv过滤之后图像断裂与破碎，因此使用膨胀，优化后续凸包的效果
  cv::dilate(
    frame_mask,
    frame_mask,
    kernel,
    cv::Point(-1, -1),
    1);


    
  /*
  ====================================
                Part 3 
                构造凸包
  ====================================
  */  

  /*定义轮廓容器：
  * - 外层 vector：保存多个轮廓
  * - 内层 vector<cv::Point>：保存一个轮廓中的所有点
  */
  std::vector<std::vector<cv::Point>> contours;

  //从二值图 frame_mask 中提取轮廓。
  cv::findContours(
    frame_mask,            //经过hsv过滤，形态学处理的二值图 frame_mask
    contours,              //轮廓容器
    cv::RETR_EXTERNAL,     //只查找最外层轮廓，不处理内部轮廓
    cv::CHAIN_APPROX_SIMPLE//压缩轮廓点，减少不必要的点
                  );

  //如果没有找到任何轮廓，直接返回“未检测到目标”的结果。
  if (contours.empty()) {
    return result;
  }

  //准备记录面积最大的轮廓：
  size_t max_index = 0; //最大轮廓的下标
  double max_area = 0.0;//当前最大面积


  /**
   * @brief 遍历所有轮廓，并计算每个轮廓的面积。
   *        如果当前轮廓面积更大，就记录它
   *        循环结束后，contours[max_index] 就是最大轮廓
   */
  for (size_t i = 0; i < contours.size(); ++i) {
    const double area = cv::contourArea(contours[i]);

    if (area > max_area) {
      max_area = area;
      max_index = i;
    }
  }

  //轮廓点少于 3 个，无法形成有效的多边形，因此返回
  if (contours[max_index].size() < 3) {
    return result;
  }

  /**
   * @brief 根据最大轮廓计算凸包
   *        凸包可以理解为：用一条橡皮筋包住目标后，
   *        橡皮筋形成的外边界。它可以去除轮廓中的凹陷部分，
   *        使轮廓更规则
   */
  std::vector<cv::Point> hull;
  cv::convexHull(contours[max_index], hull);

  // 如果凸包点少于 3 个，也无法形成有效图形，因此返回
  if (hull.size() < 3) {
    return result;
  }

  //把单个凸包包装成“多个轮廓”的格式，因为 cv::polylines() 接收的是轮廓列表。
  std::vector<std::vector<cv::Point>> hull_list{hull};

  //在 output 图像上绘制凸包
  cv::polylines(
    output,                 //output：要绘制的图像
    hull_list,              //hull_list：要绘制的轮廓
    true,                   //闭合曲线，连接首尾两个点
    cv::Scalar(0, 255, 0),  //绿色，OpenCV 使用 BGR 顺序
    2                       //线宽为 2 像素
               );

  // 根据凸包计算最小外接圆，但不绘制圆。
  cv::minEnclosingCircle(
    hull,
    result.center,
    result.radius);

  // 只绘制最小外接圆的圆心，不绘制圆本身。
  cv::circle(
    output,
    cv::Point(
      static_cast<int>(result.center.x),
      static_cast<int>(result.center.y)),
    5,
    cv::Scalar(0, 0, 255),
    -1);

  result.found = true;
  return result;
}
