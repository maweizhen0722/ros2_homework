#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iostream>

void callback(const std_msgs::msg::String::SharedPtr msg) {
    std::cout << "收到消息: " << msg->data << std::endl;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("sub_node");
    auto sub = node->create_subscription<std_msgs::msg::String>(
        "my_topic", 10, callback);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
