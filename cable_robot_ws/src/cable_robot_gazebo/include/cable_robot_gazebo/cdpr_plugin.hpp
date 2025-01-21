#ifndef CDPR_PLUGIN_HPP
#define CDPR_PLUGIN_HPP

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <gazebo_msgs/msg/link_state.hpp>
#include <cable_robot_interfaces/msg/cable_tensions.hpp>

namespace gazebo
{
class CDPRPlugin : public ModelPlugin
{
    struct Tension
    {
        ignition::math::Vector3d force;
        ignition::math::Vector3d point;
        std::string name;
    };

public:
    CDPRPlugin() = default;
    ~CDPRPlugin()
    {

    }

    void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf) override;
    void Update();

private:
    // Callback functions
    void JointCommandCallback(const sensor_msgs::msg::JointState::SharedPtr _msg)
    {
        joint_command_ = *_msg;
        command_received_ = true;
    }

    void TensionCallback(const cable_robot_interfaces::msg::CableTensions::SharedPtr msg)
{
    if (msg->cable_names.size() != msg->cable_directions.size() ||
        msg->cable_names.size() != msg->cable_tensions.size())
    {
        RCLCPP_WARN(node_->get_logger(), "Received inconsistent tension dimensions");
        return;
    }
    command_received_ = true;
    for (size_t i = 0; i < msg->cable_names.size(); ++i)
    {
        // Look for corresponding cable in command
        auto cable = std::find_if(tension_command_.begin(),
                                tension_command_.end(),
                                [&](const Tension &t)
                                { return t.name == msg->cable_names[i]; });
        if (cable != tension_command_.end())
        {
            cable->force.X() = msg->cable_directions[i].x;
            cable->force.Y() = msg->cable_directions[i].y;
            cable->force.Z() = msg->cable_directions[i].z;
            cable->force.Normalize();
            cable->force *= std::max<double>(0, msg->cable_tensions[i]);
        }
    }
}

private:
    // ROS 2 node handling
    gazebo_ros::Node::SharedPtr node_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    // Model and physics
    physics::ModelPtr model_;
    event::ConnectionPtr update_event_;
    double update_T_;

    // Joint control
    std::vector<physics::JointPtr> joints_;
    double f_max;

    // Subscribers and command handling
    bool sim_cables_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr command_subscriber_;
    rclcpp::Subscription<cable_robot_interfaces::msg::CableTensions>::SharedPtr tension_subscriber_;
    sensor_msgs::msg::JointState joint_command_;
    std::vector<Tension> tension_command_;
    bool command_received_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
    rclcpp::Publisher<gazebo_msgs::msg::LinkState>::SharedPtr pf_publisher_;
    sensor_msgs::msg::JointState joint_states_;
    gazebo_msgs::msg::LinkState pf_state_;
    rclcpp::Time t_prev_;

    // Links
    physics::LinkPtr frame_link_, platform_link_;
};

GZ_REGISTER_MODEL_PLUGIN(CDPRPlugin)

} // namespace gazebo

#endif // CDPR_PLUGIN_HPP