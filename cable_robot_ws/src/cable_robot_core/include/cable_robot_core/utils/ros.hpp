#include "../core/types.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/wrench.hpp>

namespace cable_robot_core
{
    namespace ros
    {

        /**
         * @brief Convert 3D point to ROS Point message
         * @param point 3D point to convert
         * @return geometry_msgs::Point message
         */
        geometry_msgs::msg::Point toRosMsg(const Point3D &point);

        /**
         * @brief Convert ROS Point message to 3D point
         * @param point geometry_msgs::Point message to convert
         * @return 3D point
         */
        Point3D fromRosMsg(const geometry_msgs::msg::Point &point);

        /**
         * @brief Convert Pose to ROS Pose message
         * @param pose Pose to convert
         * @return geometry_msgs::Pose message
         */
        geometry_msgs::msg::Pose toRosMsg(const Pose &pose);

        /**
         * @brief Convert ROS Pose message to Pose
         * @param pose geometry_msgs::Pose message to convert
         * @return Pose
         */
        Pose fromRosMsg(const geometry_msgs::msg::Pose &pose);

        /**
         * @brief Convert Wrench to ROS Wrench message
         * @param wrench Wrench to convert
         * @return geometry_msgs::Wrench message
         */
        geometry_msgs::msg::Wrench toRosMsg(const Wrench &wrench);

        /**
         * @brief Convert ROS Wrench message to Wrench
         * @param wrench geometry_msgs::Wrench message to convert
         * @return Wrench
         */
        Wrench fromRosMsg(const geometry_msgs::msg::Wrench &wrench);

    }
}