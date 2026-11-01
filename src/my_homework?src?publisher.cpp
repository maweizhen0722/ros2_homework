#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("pub_node");
    auto pub = node->create_publisher<std_msgs::msg::String>("my_topic", 10);

    auto timer = node->create_wall_timer(1s, [pub]() {
        auto msg = std_msgs::msg::String();
        msg.data = "Hello Navigation!";
        pub->publish(msg);
    });

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
