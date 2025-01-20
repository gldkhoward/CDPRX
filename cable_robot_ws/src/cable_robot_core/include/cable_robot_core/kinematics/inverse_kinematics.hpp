#ifndef CABLE_ROBOT_CORE_INVERSE_KINEMATICS_HPP_
#define CABLE_ROBOT_CORE_INVERSE_KINEMATICS_HPP_

#include "../core/types.hpp"
#include <optional>

namespace cable_robot_core
{

    /**
     * @brief Class for computing inverse kinematics of the cable-driven parallel robot
     * @details Implements the inverse kinematics transformation described in Section 4.2.1.
     *          For a given pose y = (r, R), computes the cable lengths l and structure matrix A.
     *          The solution always exists and is unique for any pose (r, R).
     *
     * Key equations:
     * - Cable length: li = |li| = |ai - r - Rbi| for i = 1,...,m
     * - Structure matrix: AT = [-u1^T, (b1 × u1)^T; ...; -um^T, (bm × um)^T]
     * - Jacobian: JDK = -A (from equation 4.5)
     *
     * Performance characteristics:
     * - 6 trigonometric evaluations
     * - ~242 arithmetic floating point operations
     * - Real-time capable on microcontrollers
     *
     * @note All vectors are expressed in the world frame unless otherwise specified
     */
    class InverseKinematics
    {
    public:
        /**
         * @brief Constructor
         * @param config Robot configuration containing:
         *        - ai: Anchor points on the base frame (world coordinates)
         *        - bi: Attachment points on the platform (platform coordinates)
         *        - workspace_limits: Valid ranges for position and orientation
         * @throws std::invalid_argument if configuration is invalid
         */
        explicit InverseKinematics(const RobotConfiguration &config);

        /**
         * @brief Calculate cable lengths for a given platform pose
         * @details Implements equation li = |ai - r - Rbi| for i = 1,...,m
         *          Always produces a unique solution for any valid pose.
         * @param pose Platform pose y = (r, R) where:
         *        - r: Position vector in world frame
         *        - R: Rotation matrix from platform to world frame
         * @return Vector of cable lengths [l1, ..., lm]
         * @throws std::invalid_argument if pose is invalid
         * @note Computational complexity: O(m) where m is number of cables
         */
        std::vector<double> calculateCableLengths(const Pose &pose) const;

        RobotConfiguration config_; // Robot configuration
    };

} // namespace cable_robot_core

#endif // CABLE_ROBOT_CORE_INVERSE_KINEMATICS_HPP_