#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class HelloNav2 : public rclcpp::Node
{
public:
  HelloNav2()
  : Node("hello_nav2")
  {
    timer_ = create_wall_timer(1s, [this]() {
      RCLCPP_INFO(get_logger(), "nav2_learning is running");
    });
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloNav2>());
  rclcpp::shutdown();
  return 0;
}
