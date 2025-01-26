#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench.hpp>
#include <cable_robot_interfaces/msg/platform_forces.hpp>

using std::placeholders::_1;

class ForceApplier : public rclcpp::Node
{
public:
    ForceApplier() : Node("force_applier")
    {
        // 1. Parameter Initialization
        this->declare_parameter<int>("num_attachment_points", 4);
        num_attachment_points_ = this->get_parameter("num_attachment_points").as_int();

        if (num_attachment_points_ <= 0)
        {
            RCLCPP_FATAL(this->get_logger(), "Invalid num_attachment_points: %d. Must be > 0", num_attachment_points_);
            throw std::runtime_error("Invalid num_attachment_points");
        }

        // 2. Create Publishers for Each Attachment Point
        RCLCPP_INFO(this->get_logger(), "Creating %d force publishers:", num_attachment_points_);
        for (int i = 1; i <= num_attachment_points_; ++i)
        {
            std::string topic_name = "/cable_robot/force_ap" + std::to_string(i);
            auto publisher = this->create_publisher<geometry_msgs::msg::Wrench>(topic_name, 10);
            wrench_publishers_.push_back(publisher);
            RCLCPP_INFO(this->get_logger(), " - %s", topic_name.c_str());
        }

        // 3. Subscribe to Input Forces
        input_subscriber_ = this->create_subscription<cable_robot_interfaces::msg::PlatformForces>("/cable_robot/input_forces", 10,
                                                                                                   std::bind(&ForceApplier::input_forces_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Force applier ready");
    }

private:
    // 4. Input Forces Callback
    void input_forces_callback(const cable_robot_interfaces::msg::PlatformForces::SharedPtr msg)
    {
        // Validate message size
        if (msg->forces.size() != num_attachment_points_)
        {
            RCLCPP_ERROR(this->get_logger(),
                         "Received %zu forces but expected %d. Ignoring message.",
                         msg->forces.size(), num_attachment_points_);
            return;
        }

        // Publish each wrench to corresponding topic
        for (size_t i = 0; i < msg->forces.size(); ++i)
        {
            const auto &wrench = msg->forces[i];

            // Optional: Add debug logging
            RCLCPP_DEBUG(this->get_logger(),
                         "AP%d: Force [%.2f, %.2f, %.2f] | Torque [%.2f, %.2f, %.2f]",
                         i + 1,
                         wrench.force.x, wrench.force.y, wrench.force.z,
                         wrench.torque.x, wrench.torque.y, wrench.torque.z);

            wrench_publishers_[i]->publish(wrench);
        }
    }

    // Member variables
    int num_attachment_points_;
    std::vector<rclcpp::Publisher<geometry_msgs::msg::Wrench>::SharedPtr> wrench_publishers_;
    rclcpp::Subscription<cable_robot_interfaces::msg::PlatformForces>::SharedPtr input_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<ForceApplier>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("force_applier"), "Exception: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}