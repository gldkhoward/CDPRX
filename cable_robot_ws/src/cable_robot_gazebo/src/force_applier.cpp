#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/point.hpp> // Include Point message type
#include <gazebo_msgs/srv/apply_link_wrench.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <memory>
#include <chrono>

class ForceApplier : public rclcpp::Node
{
public:
    ForceApplier() : Node("force_applier")
    {
        // Declare and get the node ID parameter
        this->declare_parameter<int>("id", 1);
        id_ = this->get_parameter("id").as_int();

        // Validate ID
        if (id_ < 1 || id_ > 4)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid ID %d. Must be between 1 and 4.", id_);
            throw std::runtime_error("Invalid node ID");
        }

        // Create subscriber for force vector
        std::string topic_name = "/cable_force_vector_" + std::to_string(id_);
        subscriber_ = this->create_subscription<geometry_msgs::msg::Vector3>(
            topic_name, 10,
            std::bind(&ForceApplier::force_callback, this, std::placeholders::_1));

        // Create Gazebo service client
        client_ = this->create_client<gazebo_msgs::srv::ApplyLinkWrench>("/gazebo/apply_link_wrench");

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for Gazebo ApplyLinkWrench service...");
        }

        RCLCPP_INFO(this->get_logger(), "Force applier %d initialized.", id_);
    }

private:
    void force_callback(const geometry_msgs::msg::Vector3::SharedPtr msg)
    {
        RCLCPP_DEBUG(this->get_logger(), "Received force: [x: %f, y: %f, z: %f]", msg->x, msg->y, msg->z);

        // Prepare the service request
        auto request = std::make_shared<gazebo_msgs::srv::ApplyLinkWrench::Request>();
        request->link_name = "attachment_point_" + std::to_string(id_);
        request->reference_frame = "world"; // Apply force in the world frame

        // Set reference point as a Point message (not Vector3)
        request->reference_point.x = 0.0;
        request->reference_point.y = 0.0;
        request->reference_point.z = 0.0;

        // Set the force and torque
        request->wrench.force = *msg;
        request->wrench.torque = geometry_msgs::msg::Vector3(); // No torque
        request->duration.sec = 10;                             // Apply force for 10 seconds

        // Call the service asynchronously
        auto future = client_->async_send_request(request);
        future.wait_for(std::chrono::seconds(1));

        // Check the result
        if (future.valid())
        {
            auto response = future.get();
            if (response->success)
            {
                RCLCPP_INFO(this->get_logger(), "Applied force to link %d", id_);
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Service error: %s", response->status_message.c_str());
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed.");
        }
    }

    int id_;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscriber_;
    rclcpp::Client<gazebo_msgs::srv::ApplyLinkWrench>::SharedPtr client_;
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