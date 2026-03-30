#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 消息回调函数：处理接收到的话题消息
void topic_callback(const std_msgs::msg::String & msg)
{
  printf("接收到消息：%s\n", msg.data.c_str());
}

int main(int argc, char * argv[])
{
  // 初始化ROS 2上下文
  rclcpp::init(argc, argv);
  // 创建订阅者节点
  auto node = rclcpp::Node::make_shared("coming");
  printf("来了来了,别催了，在路上");
  printf("\n");
  // 创建订阅者：订阅“topic”话题，队列大小10，绑定回调函数
  auto subscription = node->create_subscription<std_msgs::msg::String>(
    "winter", 10, topic_callback);
  // 持续监听消息（阻塞式运行节点）
  rclcpp::spin(node);
  // 关闭ROS 2上下文
  rclcpp::shutdown();  
  return 0;                                                                                                                                         
}