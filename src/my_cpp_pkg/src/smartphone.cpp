#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/string.hpp"
using namespace std::placeholders;
class SmartPhone : public rclcpp::Node
{
    public:
        SmartPhone() : Node("smart_phone")
        {
            subscriber_=this->create_subscription<example_interfaces::msg::String>("robot_news",10,
                std::bind(&SmartPhone::callbackRobotNews,this,std::placeholders::_1));
                RCLCPP_INFO(this->get_logger(),"Smartphone has been prepared!");
            publisher_=this->create_publisher<example_interfaces::msg::String>("notice",10);
        }
    private:
        void callbackRobotNews(const example_interfaces::msg::String::SharedPtr msg)
        {
            RCLCPP_INFO(this->get_logger(),"%s",msg->data.c_str());
            auto response_msg=example_interfaces::msg::String();
            response_msg.data=std::string("hey!I just receive the message:")+msg->data;
            publisher_->publish(response_msg);
            

        }
        rclcpp::Subscription<example_interfaces::msg::String>::SharedPtr subscriber_;
        rclcpp::Publisher<example_interfaces::msg::String>::SharedPtr publisher_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc,argv);
    auto node = std::make_shared<SmartPhone>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}