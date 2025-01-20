#include "../../include/cable_robot_core/core/types.hpp"
#include "../../include/cable_robot_core/utils/utils.hpp"
#include <stdexcept>

namespace cable_robot_core
{
    namespace utils
    {

        //////////////////////////////////////////////////////////
        ///////////////// INVERSE KINEMATICS /////////////////////
        //////////////////////////////////////////////////////////

        /**
         * @brief Check if a position is within workspace bounds
         * @param position Position vector to check
         * @param workspace Workspace parameters containing limits
         * @return true if position is valid
         */
        bool isPositionInWorkspace(const Vector3 &position,
                                   const WorkspaceParameters &workspace)
        {
            return position.x() >= workspace.min_x && position.x() <= workspace.max_x &&
                   position.y() >= workspace.min_y && position.y() <= workspace.max_y &&
                   position.z() >= workspace.min_z && position.z() <= workspace.max_z;
        }

        /**
         * @brief Check if orientation angles are within workspace bounds
         * @param orientation Rotation matrix to check
         * @param workspace Workspace parameters containing limits
         * @return true if orientation is valid
         */
        bool isOrientationInWorkspace(const Matrix3 &orientation,
                                      const WorkspaceParameters &workspace)
        {
            // Extract Euler angles from rotation matrix
            Vector3 euler = rotationMatrixToEuler(orientation);

            return euler.x() >= workspace.min_roll && euler.x() <= workspace.max_roll &&
                   euler.y() >= workspace.min_pitch && euler.y() <= workspace.max_pitch &&
                   euler.z() >= workspace.min_yaw && euler.z() <= workspace.max_yaw;
        }

        /**
         * @brief Extract roll, pitch, yaw from rotation matrix
         * @param R Rotation matrix
         * @return Vector3 containing roll, pitch, yaw angles in radians
         */
        Vector3 rotationMatrixToRPY(const Matrix3 &R)
        {
            // Extract roll, pitch, yaw angles from rotation matrix
            double roll = atan2(R(2, 1), R(2, 2));
            double pitch = atan2(-R(2, 0), sqrt(R(2, 1) * R(2, 1) + R(2, 2) * R(2, 2)));
            double yaw = atan2(R(1, 0), R(0, 0));

            return Vector3(roll, pitch, yaw);
        }

        /**
         * @brief Calculate unit vector and length for a cable
         * @param anchor_point Cable anchor point on frame (world frame)
         * @param platform_point Cable attachment point on platform (world frame)
         * @return std::pair containing unit vector and length
         */
        std::pair<Vector3, double> calculateCableVectorAndLength(
            const Point3D &anchor_point,
            const Point3D &platform_point)
        {
            // Calculate cable vector and length
            Vector3 cable_vector = pointToVector(platform_point) - pointToVector(anchor_point);
            double cable_length = cable_vector.norm();

            // Normalize cable vector
            cable_vector.normalize();

            return std::make_pair(cable_vector, cable_length);
        }

        Vector3 pointToVector(const Point3D &point)
        {
            return Vector3(point.x, point.y, point.z);
        }

        /**
         * @brief Transform platform attachment points from platform frame to world frame
         * @param attachment_point Point in platform frame
         * @param pose Current platform pose
         * @return Point3D in world frame
         */
        Point3D transformToWorldFrame(const Point3D &attachment_point,
                                      const Pose &pose)
        {
            // Convert Point3D to Vector3
            Vector3 p_local(attachment_point.x, attachment_point.y, attachment_point.z);

            // Apply transformation: p_world = r + R * p_local
            Vector3 p_world = pose.position + pose.orientation * p_local;

            return Point3D(p_world.x(), p_world.y(), p_world.z());
        }

        //////////////////////////////////////////////////////////
        ///////////////// FORWARD KINEMATICS /////////////////////
        //////////////////////////////////////////////////////////

        //////////////////////////////////////////////////////////
        ///////////////////// TODOS //////////////////////////////
        //////////////////////////////////////////////////////////

        bool isWithinWorkspaceLimits(
            const Pose &pose,
            const WorkspaceParameters &limits)
        {
            // TODO: Implement workspace limit validation
            return false;
        }

        // Transform point from platform frame to world frame
        Vector3 transformToWorld(const Point3D &point, const Pose &pose)
        {
            // Convert Point3D to Vector3
            Vector3 p_local(point.x, point.y, point.z);

            // Apply transformation: p_world = r + R * p_local
            return pose.position + pose.orientation * p_local;
        }

        Point3D transformToPlatform(const Point3D &point, const Pose &pose)
        {
            // TODO: Implement p_platform = R^T * (p_world - r) transformation
            return Point3D();
        }

        MatrixX calculateStructureMatrix(
            const std::vector<Point3D> &platform_points,
            const std::vector<Point3D> &frame_points,
            const Pose &platform_pose)
        {
            // TODO: Implement structure matrix calculation A^T = [-u1^T, (b1 × u1)^T; ...; -um^T, (bm × um)^T]
            return MatrixX();
        }

        std::vector<Vector3> calculateCableUnitVectors(
            const std::vector<Point3D> &platform_points,
            const std::vector<Point3D> &frame_points,
            const Pose &platform_pose)
        {
            // TODO: Implement unit vector calculation for all cables
            return std::vector<Vector3>();
        }

        std::vector<Vector3> getCableUnitVectors(const Pose &pose)
        {
            // TODO: Implement cable unit vector calculation ui = (ai - r - Rbi)/|ai - r - Rbi|
            return std::vector<Vector3>();
        }

        Vector3 calculateCableVector(const Pose &pose, std::size_t cable_idx)
        {
            // TODO: Implement cable vector calculation li = ai - r - Rbi
            return Vector3::Zero();
        }

        Point3D vectorToPoint(const Vector3 &vec)
        {
            return Point3D(vec.x(), vec.y(), vec.z());
        }

        Vector3 rotationMatrixToEuler(const Matrix3 &rotation)
        {
            // TODO: Implement rotation matrix to Euler angles conversion
            return Vector3::Zero();
        }

        Matrix3 eulerToRotationMatrix(const Vector3 &euler)
        {
            // TODO: Implement Euler angles to rotation matrix conversion
            return Matrix3::Identity();
        }

        double calculateDistance(const Point3D &p1, const Point3D &p2)
        {
            // TODO: Implement 3D distance calculation
            return 0.0;
        }

        bool areCableLengthsValid(
            const std::vector<double> &lengths,
            const std::vector<CableParameters> &cable_params)
        {
            // TODO: Implement cable length validation
            return false;
        }

        bool areCableTensionsValid(
            const std::vector<double> &tensions,
            const std::vector<CableParameters> &cable_params)
        {
            // TODO: Implement cable tension validation
            return false;
        }

        Point3D calculateWorldCoM(
            const Pose &pose,
            const PlatformParameters &platform_params)
        {
            // TODO: Implement world frame center of mass calculation
            return Point3D();
        }

        std::optional<Pose> findClosestValidPose(
            const Pose &pose,
            const WorkspaceParameters &limits)
        {
            // TODO: Implement closest valid pose finder
            return std::nullopt;
        }

        MatrixX calculateWrenchMatrix(
            const std::vector<Point3D> &platform_points,
            const std::vector<Point3D> &world_points)
        {
            // TODO: Implement wrench matrix calculation
            return MatrixX();
        }

        Matrix6 calculateJacobian(const Pose &pose)
        {
            // TODO: Implement Jacobian calculation JDK = -A
            return Matrix6::Zero();
        }

        bool isPoseKinematicallyFeasible(
            const Pose &pose,
            const RobotConfiguration &config)
        {
            // TODO: Implement kinematic feasibility check
            return false;
        }

        WorkspaceMetrics calculateWorkspaceMetrics(
            const Pose &pose,
            const MatrixX &structure_matrix)
        {
            // TODO: Implement workspace metrics calculation
            return WorkspaceMetrics();
        }

        Pose interpolatePose(
            const Pose &start_pose,
            const Pose &end_pose,
            double t)
        {
            // TODO: Implement pose interpolation
            return Pose();
        }

        Matrix6 calculateWorldInertia(
            const Pose &pose,
            const PlatformParameters &platform_params)
        {
            // TODO: Implement world frame inertia matrix calculation
            return Matrix6::Zero();
        }

    } // namespace utils
} // namespace cable_robot_core