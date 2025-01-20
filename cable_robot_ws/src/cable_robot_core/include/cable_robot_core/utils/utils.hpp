#ifndef CABLE_ROBOT_CORE_UTILS_HPP_
#define CABLE_ROBOT_CORE_UTILS_HPP_

#include "../core/types.hpp"
#include <cmath>
#include <vector>
#include <optional>

namespace cable_robot_core
{
    namespace utils
    {
        /**
         * @brief Check if a pose is within the workspace limits
         * @param pose Robot pose to check
         * @param limits Workspace limits
         * @return true if within limits, false otherwise
         */
        bool isWithinWorkspaceLimits(
            const Pose &pose,
            const WorkspaceParameters &limits);

        /**
         * @brief Transform point from platform to world frame
         * @details Applies transform: p_world = r + R * p_platform
         * @param point Point coordinates in platform frame
         * @param pose Current platform pose
         * @return Point coordinates in world frame
         */
        Vector3 transformToWorld(const Point3D &point, const Pose &pose);

        /**
         * @brief Transform a point from world frame to platform frame
         * @param point Point in world frame
         * @param pose Current platform pose
         * @return Point in platform frame
         */
        Point3D transformToPlatform(const Point3D &point, const Pose &pose);

        /**
         * @brief Calculate the structure matrix for the cable robot
         * @param platform_points Cable attachment points on platform in platform frame
         * @param frame_points Cable attachment points on frame in world frame
         * @param platform_pose Current platform pose
         * @return Structure matrix A^T mapping cable tensions to platform wrench
         */
        MatrixX calculateStructureMatrix(
            const std::vector<Point3D> &platform_points,
            const std::vector<Point3D> &frame_points,
            const Pose &platform_pose);

        /**
         * @brief Calculate unit vectors for all cables
         * @return Vector of unit vectors for each cable
         */
        std::vector<Vector3> calculateCableUnitVectors(
            const std::vector<Point3D> &platform_points,
            const std::vector<Point3D> &frame_points,
            const Pose &platform_pose);

        /**
         * @brief Get cable unit vectors for given pose
         * @details Unit vectors ui = (ai - r - Rbi)/|ai - r - Rbi|
         *          pointing from platform attachment to base anchor
         * @param pose Platform pose
         * @return Vector of unit vectors [u1, ..., um]
         * @throws std::invalid_argument if pose is invalid
         */
        std::vector<Vector3> getCableUnitVectors(const Pose &pose);

        /**
         * @brief Calculate vector from platform point to anchor point
         * @details Computes li = ai - r - Rbi for given cable
         * @param pose Platform pose
         * @param cable_idx Cable index in range [0, m-1]
         * @return Vector from platform attachment to anchor point
         * @throws std::out_of_range if cable_idx invalid
         */
        Vector3 calculateCableVector(const Pose &pose, std::size_t cable_idx);

        /**
         * @brief Check if a position is within workspace bounds
         * @param position Position vector to check
         * @param workspace Workspace parameters containing limits
         * @return true if position is valid
         */
        bool isPositionInWorkspace(const Vector3 &position,
                                   const WorkspaceParameters &workspace);

        /**
         * @brief Check if orientation angles are within workspace bounds
         * @param orientation Rotation matrix to check
         * @param workspace Workspace parameters containing limits
         * @return true if orientation is valid
         */
        bool isOrientationInWorkspace(const Matrix3 &orientation,
                                      const WorkspaceParameters &workspace);

        /**
         * @brief Extract roll, pitch, yaw from rotation matrix
         * @param R Rotation matrix
         * @return Vector3 containing roll, pitch, yaw angles in radians
         */
        Vector3 rotationMatrixToRPY(const Matrix3 &R);

        /**
         * @brief Transform platform attachment points from platform frame to world frame
         * @param attachment_point Point in platform frame
         * @param pose Current platform pose
         * @return Point3D in world frame
         */
        Point3D transformToWorldFrame(const Point3D &attachment_point,
                                      const Pose &pose);

        /**
         * @brief Calculate unit vector and length for a cable
         * @param anchor_point Cable anchor point on frame (world frame)
         * @param platform_point Cable attachment point on platform (world frame)
         * @return std::pair containing unit vector and length
         */
        std::pair<Vector3, double> calculateCableVectorAndLength(
            const Point3D &anchor_point,
            const Point3D &platform_point);
        /**
         * @brief Validate cable parameters
         * @param params Cable parameters to validate
         * @return std::optional<std::string> containing error message if invalid
         */
        std::optional<std::string> validateCableParameters(
            const CableParameters &params);

        /**
         * @brief Convert Vector3 to Point3D
         */
        Point3D vectorToPoint(const Vector3 &vec);

        /**
         * @brief Convert Point3D to Vector3
         */
        Vector3 pointToVector(const Point3D &point);

        /**
         * @brief Extract Euler angles (roll, pitch, yaw) from rotation matrix
         * @param rotation Rotation matrix
         * @return Vector3 containing (roll, pitch, yaw) in radians
         */
        Vector3 rotationMatrixToEuler(const Matrix3 &rotation);

        /**
         * @brief Create rotation matrix from Euler angles
         * @param euler Vector3 containing (roll, pitch, yaw) in radians
         * @return Rotation matrix
         */
        Matrix3 eulerToRotationMatrix(const Vector3 &euler);

        /**
         * @brief Calculate distance between two 3D points
         */
        double calculateDistance(const Point3D &p1, const Point3D &p2);

        /**
         * @brief Check if cable lengths are within allowed limits
         * @param lengths Vector of cable lengths
         * @param cable_params Vector of cable parameters
         * @return true if all lengths are within limits
         */
        bool areCableLengthsValid(
            const std::vector<double> &lengths,
            const std::vector<CableParameters> &cable_params);

        /**
         * @brief Check if cable tensions are within allowed limits
         * @param tensions Vector of cable tensions
         * @param cable_params Vector of cable parameters
         * @return true if all tensions are within limits
         */
        bool areCableTensionsValid(
            const std::vector<double> &tensions,
            const std::vector<CableParameters> &cable_params);

        /**
         * @brief Calculate the center of mass in world frame
         * @param pose Platform pose
         * @param platform_params Platform parameters
         * @return Center of mass position in world frame
         */
        Point3D calculateWorldCoM(
            const Pose &pose,
            const PlatformParameters &platform_params);

        /**
         * @brief Find closest valid pose within workspace limits
         * @param pose Current pose
         * @param limits Workspace limits
         * @return Optional containing the closest valid pose, or empty if cannot be found
         */
        std::optional<Pose> findClosestValidPose(
            const Pose &pose,
            const WorkspaceParameters &limits);

        /**
         * @brief Calculate wrench matrix for given cable configuration
         * @param platform_points Cable attachment points on platform
         * @param world_points Cable attachment points in world frame
         * @return Structure matrix mapping cable tensions to resultant wrench
         */
        MatrixX calculateWrenchMatrix(
            const std::vector<Point3D> &platform_points,
            const std::vector<Point3D> &world_points);

        /**
         * @brief Calculate velocity mapping matrix (Jacobian)
         * @details Implements equation JDK = -A from Section 4.2.3
         *          Maps platform twist to cable velocities: l̇ = J ẏ
         * @param pose Current platform pose
         * @return Jacobian matrix J ∈ ℝ^(m×6)
         * @throws std::invalid_argument if pose is invalid
         */
        Matrix6 calculateJacobian(const Pose &pose);

        /**
         * @brief Check if pose is kinematically feasible
         * @param pose Platform pose to check
         * @param config Robot configuration
         * @return true if pose is feasible
         */
        bool isPoseKinematicallyFeasible(
            const Pose &pose,
            const RobotConfiguration &config);

        /**
         * @brief Calculate workspace metrics for a given pose
         * @param pose Platform pose
         * @param structure_matrix Structure matrix at that pose
         * @return WorkspaceMetrics containing various workspace quality measures
         */
        WorkspaceMetrics calculateWorkspaceMetrics(
            const Pose &pose,
            const MatrixX &structure_matrix);

        /**
         * @brief Interpolate between two poses
         * @param start_pose Starting pose
         * @param end_pose Target pose
         * @param t Interpolation parameter [0,1]
         * @return Interpolated pose
         */
        Pose interpolatePose(
            const Pose &start_pose,
            const Pose &end_pose,
            double t);

        /**
         * @brief Calculate platform inertia matrix in world frame
         * @param pose Current platform pose
         * @param platform_params Platform parameters
         * @return 6x6 spatial inertia matrix in world frame
         */
        Matrix6 calculateWorldInertia(
            const Pose &pose,
            const PlatformParameters &platform_params);

    } // namespace utils
} // namespace cable_robot_core

#endif // CABLE_ROBOT_CORE_UTILS_HPP_