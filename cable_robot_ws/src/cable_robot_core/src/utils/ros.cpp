#include "../../include/cable_robot_core/utils/ros.hpp"

namespace cable_robot_core
{
    namespace ros
    {

        // Convert Point3D to ROS Point message
        geometry_msgs::msg::Point toRosMsg(const Point3D &point)
        {
            geometry_msgs::msg::Point msg;
            msg.x = point.x;
            msg.y = point.y;
            msg.z = point.z;
            return msg;
        }

        // Convert ROS Point message to Point3D
        Point3D fromRosMsg(const geometry_msgs::msg::Point &point)
        {
            return Point3D(point.x, point.y, point.z);
        }

        // Convert Pose to ROS Pose message
        geometry_msgs::msg::Pose tpRosMsg(const Pose &pose)
        {
            geometry_msgs::msg::Pose msg;

            // Convert position
            msg.position.x = pose.position.x();
            msg.position.y = pose.position.y();
            msg.position.z = pose.position.z();

            // Convert orientation (rotation matrix to quaternion)
            Eigen::Quaterniond quat(pose.orientation);
            msg.orientation.x = quat.x();
            msg.orientation.y = quat.y();
            msg.orientation.z = quat.z();
            msg.orientation.w = quat.w();

            return msg;
        }

        // Convert ROS Pose message to Pose
        Pose fromRosMsg(const geometry_msgs::msg::Pose &pose)
        {
            Pose result;

            // Convert position
            result.position << pose.position.x, pose.position.y, pose.position.z;

            // Convert orientation (quaternion to rotation matrix)
            Eigen::Quaterniond quat(
                pose.orientation.w,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z);
            result.orientation = quat.toRotationMatrix();

            return result;
        }

        // Convert Wrench to ROS Wrench message
        geometry_msgs::msg::Wrench toRosMsg(const Wrench &wrench)
        {
            geometry_msgs::msg::Wrench msg;

            // Convert force
            msg.force.x = wrench.force.x();
            msg.force.y = wrench.force.y();
            msg.force.z = wrench.force.z();

            // Convert moment
            msg.torque.x = wrench.moment.x();
            msg.torque.y = wrench.moment.y();
            msg.torque.z = wrench.moment.z();

            return msg;
        }

        // Convert ROS Wrench message to Wrench
        Wrench fromRosMsg(const geometry_msgs::msg::Wrench &wrench)
        {
            Wrench result;

            // Convert force
            result.force << wrench.force.x, wrench.force.y, wrench.force.z;

            // Convert moment
            result.moment << wrench.torque.x, wrench.torque.y, wrench.torque.z;

            return result;
        }

    } // namespace ros
} // namespace cable_robot_core