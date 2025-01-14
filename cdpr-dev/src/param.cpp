#include <rclcpp/rclcpp.hpp>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    // Initialize ROS2
    rclcpp::init(argc, argv);

    // Create node
    auto node = rclcpp::Node::make_shared("gna");

    // Declare and get parameter with default value
    bool sim = node->declare_parameter<bool>("model.sim_cables", false);
    std::cout << sim << std::endl;

    // Spin node (not needed in this simple case, but good practice)
    rclcpp::spin(node);

    // Cleanup
    rclcpp::shutdown();

    return 0;
}