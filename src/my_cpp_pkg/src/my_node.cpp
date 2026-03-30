#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 用局部变量存计数，代替类的成员变量
size_t count_ = 0;

int main(int argc, char * argv[]) 
{
    printf("来打龙蝇xdm来打龙蝇！！！");
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("hit_the_dragonfly");  // 节点名和原来一致
  auto publisher = node->create_publisher<std_msgs::msg::String>("winter", 10);  // 话题和原来一致

  // 定时器+发布逻辑（500ms发一次，和原来的500ms一致）
  auto timer = node->create_wall_timer(
    std::chrono::milliseconds(500),  // 500ms=0.5秒，和原来的500ms对应
    [publisher]() {
      auto msg = std_msgs::msg::String();
      msg.data = "龙蝇墙建好没有，我已经问了" + std::to_string(count_++)+"遍了！";  // 保留计数功能
      publisher->publish(msg);
    }
  );

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}