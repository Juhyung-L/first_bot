#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "extended_kalman_filter/filter_node.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FilterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}