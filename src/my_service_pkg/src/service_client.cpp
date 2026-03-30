#include <iostream>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using namespace std::chrono_literals;

class ClientNode : public rclcpp::Node
{
public:
    ClientNode() : Node("my_client")
    {
        client_ = this->create_client<example_interfaces::srv::AddTwoInts>(
            "add_two_ints"
        );
    }

    // 对外接口：由输入线程调用
    void send_request(long a, long b)
    {
        if (!client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Service not available");
            return;
        }

        auto request =
            std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        RCLCPP_INFO(
            this->get_logger(),
            "Sending request: a=%ld, b=%ld",
            a, b
        );

        client_->async_send_request(
            request,
            std::bind(
                &ClientNode::handle_response,
                this,
                std::placeholders::_1
            )
        );
    }

private:
    // 仅由 ROS executor 调用
    void handle_response(
        rclcpp::Client<
            example_interfaces::srv::AddTwoInts
        >::SharedFuture future)
    {
        auto result = future.get();
        RCLCPP_INFO(
            this->get_logger(),
            "Response received: sum=%ld",
            result->sum
        );
    }

    rclcpp::Client<
        example_interfaces::srv::AddTwoInts
    >::SharedPtr client_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ClientNode>();

    // 输入线程：只负责阻塞输入
    std::thread input_thread([node]() {
        while (rclcpp::ok()) {
            long a, b;
            std::cout <<"\n"<< "Enter two integers: ";
            if (!(std::cin >> a >> b)) {
                break;
            }
            node->send_request(a, b);
        }
    });

    // ROS 主循环
    rclcpp::spin(node);

    // 退出处理
    if (input_thread.joinable()) {
        input_thread.join();
    }

    rclcpp::shutdown();
    return 0;
}