#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class Server: public rclcpp::Node
{
public:
    Server() : Node("my_server")
    {
        // 创建服务
        service_ = this->create_service<example_interfaces::srv::AddTwoInts>(
            "add_two_ints",
            std::bind(&Server::handle_service, this, std::placeholders::_1, std::placeholders::_2)
        );
        RCLCPP_INFO(this->get_logger(), "AddTwoInts server is ready!");
    }

private:
    void handle_service(
        const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
        std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response
    )
    {
        // 打印请求参数，修改为 %ld 适配 long 类型
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld", request->a, request->b);
        
        // 计算并返回和
        response->sum = request->a + request->b;
        
        // 打印响应结果，修改为 %ld 适配 long 类型
        RCLCPP_INFO(this->get_logger(), "Sending response: sum=%ld", response->sum);
    }

    rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr service_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Server>();
    rclcpp::spin(node);  // 让节点保持运行
    rclcpp::shutdown();  // 关闭ROS 2
    return 0;
}
