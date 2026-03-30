cmake_minimum_required(VERSION 3.8)
project(yolo_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection.msg"                     # 两个消息文件
  "msg/DetectionArray.msg"
  DEPENDENCIES std_msgs                 # 消息里用到了哪个标准包？
)

ament_package()