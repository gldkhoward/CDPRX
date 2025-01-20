#ifndef CABLE_ROBOT_CORE_TYPES_HPP_
#define CABLE_ROBOT_CORE_TYPES_HPP_

#include <array>
#include <vector>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace cable_robot_core
{

    using Scalar = double;
    using Vector3 = Eigen::Vector3d;
    using Matrix3 = Eigen::Matrix3d;
    using Matrix6 = Eigen::Matrix<double, 6, 6>;
    using VectorX = Eigen::VectorXd;
    using MatrixX = Eigen::MatrixXd;

    /**
     * @brief Structure to represent a 3D point
     */
    struct Point3D
    {
        double x;
        double y;
        double z;

        Point3D() : x(0.0), y(0.0), z(0.0) {}
        Point3D(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    };

    /**
     * @brief Structure to represent platform pose (position and orientation)
     */
    struct Pose
    {
        Vector3 position;    // Platform center position
        Matrix3 orientation; // Rotation matrix representing orientation

        Pose() : position(Vector3::Zero()), orientation(Matrix3::Identity()) {}
    };

    /**
     * @brief Structure to represent a wrench (force and moment)
     */
    struct Wrench
    {
        Vector3 force;
        Vector3 moment;

        Wrench() : force(Vector3::Zero()), moment(Vector3::Zero()) {}
    };

    /**
     * @brief Structure containing cable parameters
     */
    struct CableParameters
    {
        double stiffness;   // Cable stiffness coefficient
        double damping;     // Cable damping coefficient
        double min_length;  // Minimum allowable length
        double max_length;  // Maximum allowable length
        double min_tension; // Minimum allowable tension
        double max_tension; // Maximum allowable tension
        double diameter;    // Cable diameter
    };

    /**
     * @brief Structure containing platform parameters
     */
    struct PlatformParameters
    {
        double mass;                            // Platform mass
        Vector3 com_offset;                     // Center of mass offset from geometric center
        Matrix3 inertia;                        // Inertia matrix
        std::vector<Point3D> attachment_points; // Cable attachment points in platform frame
    };

    /**
     * @brief Structure containing pillar paramters
     * @details Pillar is a vertical structure that supports the frame
     */
    struct PillarParameters
    {
        double mass;                            // Pillar mass
        Vector3 com_offset;                     // Center of mass offset from geometric center
        Matrix3 inertia;                        // Inertia matrix
        std::vector<Point3D> attachment_points; // Cable attachment points in pillar frame
        Vector3 dimensions;                     // Pillar dimensions (width, length, height)
    };

    /**
     * @brief Structure containing frame parameters
     */
    struct FrameParameters
    {
        std::vector<PillarParameters> pillars; // Pillar parameters
        // Dimensions of the frame calculated from the pillar dimensions
        Vector3 dimensions;

        FrameParameters() : dimensions(Vector3::Zero()) {}
    };

    /**
     * @brief Workspace Parameters
     *
     */
    struct WorkspaceParameters
    {
        double min_x, max_x;
        double min_y, max_y;
        double min_z, max_z;
        double min_roll, max_roll;
        double min_pitch, max_pitch;
        double min_yaw, max_yaw;
    };

    /**
     * @brief Complete robot configuration
     */
    struct RobotConfiguration
    {
        std::string robot_name;
        std::size_t num_cables;
        PlatformParameters platform;
        FrameParameters frame;
        std::vector<CableParameters> cables;
        WorkspaceParameters workspace_limits;
    };

    /**
     * @brief Structure representing the current state of the robot
     */
    struct RobotState
    {
        Pose platform_pose;                 // Current platform pose
        std::vector<double> cable_lengths;  // Current cable lengths
        std::vector<double> cable_tensions; // Current cable tensions
        Vector3 platform_velocity;          // Linear velocity
        Vector3 platform_angular_velocity;  // Angular velocity

        RobotState(std::size_t num_cables) : cable_lengths(num_cables, 0.0),
                                             cable_tensions(num_cables, 0.0),
                                             platform_velocity(Vector3::Zero()),
                                             platform_angular_velocity(Vector3::Zero()) {}
    };

    /**
     * @brief Structure for workspace analysis results
     */
    struct WorkspaceMetrics
    {
        bool is_wrench_closure;
        bool is_wrench_feasible;
        double condition_number;
        double min_singular_value;
        double max_singular_value;
        double tension_factor;
    };

    /**
     * @brief Enumeration for different workspace types
     */
    enum class WorkspaceType
    {
        WRENCH_CLOSURE,
        WRENCH_FEASIBLE,
        STATIC_FEASIBLE,
        DYNAMIC_FEASIBLE
    };

} // namespace cable_robot_core

#endif // CABLE_ROBOT_CORE_TYPES_HPP_