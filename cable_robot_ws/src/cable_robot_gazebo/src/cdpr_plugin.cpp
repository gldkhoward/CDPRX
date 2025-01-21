#include "../include/cable_robot_gazebo/cdpr_plugin.hpp"
#include "cable_robot_interfaces/msg/cable_tensions.hpp"

#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <gazebo_ros/conversions/geometry_msgs.hpp>
#include <algorithm>
#include <string>
#include <vector>

namespace gazebo
{

void CDPRPlugin::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
    model_ = _model;
    node_ = gazebo_ros::Node::Get(_sdf);
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor_->add_node(node_);
    t_prev_ = node_->get_clock()->now();

    joints_.clear();
    tension_command_.clear();

    node_->declare_parameter("sim_cables", false);
    node_->get_parameter("sim_cables", sim_cables_);

    if (model_->GetJointCount() != 0 && sim_cables_)
    {
        rclcpp::SubscriptionOptions ops;
        ops.callback_group = callback_group_;
        command_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
            "cable_command", 10,
            std::bind(&CDPRPlugin::JointCommandCallback, this, std::placeholders::_1),
            ops);
        command_received_ = false;

        std::vector<std::string> joint_names;
        physics::JointPtr joint;

        for (unsigned int i = 0; i < model_->GetJointCount(); ++i)
        {
            joint = model_->GetJoints()[i];
            std::string name = joint->GetName();
            if (name.find("cable") == 0)
            {
                joints_.push_back(joint);
                joint_names.push_back(name);
                f_max = joint->GetEffortLimit(0);
            }
        }

        joint_state_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("cable_states", 10);
        joint_states_.name = joint_names;
        joint_states_.position.resize(joints_.size());
        joint_states_.velocity.resize(joints_.size());
        joint_states_.effort.resize(joints_.size());
    }
    else
    {
        node_->declare_parameter("points", std::vector<double>{});
        std::vector<std::vector<double>> points;
        try {
            auto param = node_->get_parameter("points");
            // ... parse into points ...
        } catch (const rclcpp::exceptions::ParameterNotDeclaredException &) {
            RCLCPP_WARN(node_->get_logger(), "Points parameter not found, using defaults");
        }

        for (size_t i = 0; i < points.size(); ++i)
        {
            if (points[i].size() == 3)
            {
                Tension t;
                t.name = "cable" + std::to_string(i);
                t.point.X() = points[i][0];
                t.point.Y() = points[i][1];
                t.point.Z() = points[i][2];
                tension_command_.push_back(t);
            }
        }

        rclcpp::SubscriptionOptions ops;
        ops.callback_group = callback_group_;
        tension_subscriber_ = node_->create_subscription<cable_robot_interfaces::msg::CableTensions>(
            "cable_command", 10,
            std::bind(&CDPRPlugin::TensionCallback, this, std::placeholders::_1),
            ops);
        command_received_ = false;
    }

    for (const auto &link : model_->GetLinks())
    {
        if (link->GetName() == "frame")
            frame_link_ = link;
        else if (link->GetName() == "platform")
            platform_link_ = link;
    }

    pf_publisher_ = node_->create_publisher<gazebo_msgs::msg::LinkState>("pf_state", 10);
    pf_state_.link_name = "platform";
    pf_state_.reference_frame = "frame";

    update_T_ = _sdf->Get<double>("updateRate", 0.0).first;
    if (update_T_ > 0.0) update_T_ = 1.0 / update_T_;

    update_event_ = event::Events::ConnectWorldUpdateBegin(std::bind(&CDPRPlugin::Update, this));
    std::thread([this]() { executor_->spin(); }).detach();

    RCLCPP_INFO(node_->get_logger(), "Started CDPR Plugin for %s", _model->GetName().c_str());
}

void CDPRPlugin::Update()
{
    executor_->spin_some();

    if (command_received_)
    {
        if (sim_cables_)
        {
            for (size_t i = 0; i < joint_command_.name.size(); ++i)
            {
                auto it = std::find_if(joints_.begin(), joints_.end(),
                    [&](const physics::JointPtr& joint) {
                        return joint->GetName() == joint_command_.name[i];
                    });
                if (it != joints_.end())
                {
                    if (joint_command_.effort[i] > 0)
                    {
                        (*it)->SetForce(0, std::min(joint_command_.effort[i], f_max));
                    }
                }
            }
        }
        else
        {
            auto rot = platform_link_->WorldPose().Rot();
            for (const auto &t : tension_command_)
            {
                platform_link_->AddForceAtRelativePosition(rot * t.force, t.point);
            }
        }
    }

    auto current_time = node_->get_clock()->now();
    if ((current_time - t_prev_).seconds() > update_T_ && !joints_.empty())
    {
        t_prev_ = current_time;
        joint_states_.header.stamp = current_time;
        for (size_t i = 0; i < joints_.size(); ++i)
        {
            joint_states_.position[i] = joints_[i]->Position(0);
            joint_states_.velocity[i] = joints_[i]->GetVelocity(0);
            joint_states_.effort[i] = joints_[i]->GetForce(0);
        }
        joint_state_publisher_->publish(joint_states_);
    }

    auto pf_pose = platform_link_->WorldPose() - frame_link_->WorldPose();
    geometry_msgs::msg::Point pos_msg = gazebo_ros::Convert<geometry_msgs::msg::Point>(pf_pose.Pos());
    geometry_msgs::msg::Quaternion quat_msg = gazebo_ros::Convert<geometry_msgs::msg::Quaternion>(pf_pose.Rot());

    pf_state_.pose.position = pos_msg;
    pf_state_.pose.orientation = quat_msg;

    auto lin_vel = pf_pose.Rot().RotateVector(platform_link_->RelativeLinearVel());
    auto ang_vel = pf_pose.Rot().RotateVector(platform_link_->RelativeAngularVel());

    pf_state_.twist.linear = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(lin_vel);
    pf_state_.twist.angular = gazebo_ros::Convert<geometry_msgs::msg::Vector3>(ang_vel);

    pf_publisher_->publish(pf_state_);
}

} // namespace gazebo