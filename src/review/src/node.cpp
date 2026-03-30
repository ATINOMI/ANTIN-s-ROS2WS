#include "rclcpp/rclcpp.hpp"

class WeNode : public rclcpp::Node
{
public:
  // 传入节点名称（比如 "wnode"）初始化基类
  WeNode() : rclcpp::Node("wenode"),counter(0)
   {
    RCLCPP_INFO(this->get_logger(),"Let Coding!");
    timer=this->create_wall_timer(std::chrono::seconds(1),std::bind(&WeNode::timerCallback,this));
   }
private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(),"Come On!%d",counter);
        counter++;
    }
   rclcpp::TimerBase::SharedPtr timer;
   int counter;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<WeNode>();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
