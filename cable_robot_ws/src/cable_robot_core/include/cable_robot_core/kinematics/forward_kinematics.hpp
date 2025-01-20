
#ifndef CABLE_ROBOT_CORE_FORWARD_KINEMATICS_HPP_
#define CABLE_ROBOT_CORE_FORWARD_KINEMATICS_HPP_

#include "cable_robot_core/core/types.hpp"
#include <optional>

namespace cable_robot_core
{

    /**
     * @brief Class for computing forward kinematics of the cable robot
     * @details Calculates platform pose given cable lengths using numerical optimization
     */
    class ForwardKinematics
    {
    public:
        /**
         * @brief Constructor
         * @param config Robot configuration containing geometry and parameters
         */
        explicit ForwardKinematics(const RobotConfiguration &config);

        /**
         * @brief Calculate platform pose from cable lengths
         * @param cable_lengths Vector of current cable lengths
         * @param initial_guess Initial guess for platform pose (optional)
         * @return Empty pose if optimization fails, otherwise computed platform pose
         */
        std::optional<Pose> calculatePose(const std::vector<double> &cable_lengths, const std::optional<Pose> &initial_guess = std::nullopt);

        /**
         * @brief Set optimization parameters
         * @param max_iterations Maximum number of iterations for numerical solver
         * @param tolerance Convergence tolerance
         */
        void setOptimizationParams(int max_iterations, double tolerance);

        /**
         * @brief Get the last computation error
         * @return Error value from last computation
         */
        double getLastError() const;

        /**
         * @brief Check if the computed pose is valid
         * @param pose Platform pose to validate
         * @param cable_lengths Current cable lengths
         * @return true if pose is valid, false otherwise
         */
        bool validatePose(const Pose &pose, const std::vector<double> &cable_lengths) const;

    private:
        /**
         * @brief Calculate residual error for current pose estimate
         * @param pose Current pose estimate
         * @param cable_lengths Target cable lengths
         * @return Error vector
         */
        VectorX calculateResidual(const Pose &pose, const std::vector<double> &cable_lengths) const;

        /**
         * @brief Calculate Jacobian matrix for optimization
         * @param pose Current pose estimate
         * @return Jacobian matrix
         */
        MatrixX calculateJacobian(const Pose &pose) const;

        /**
         * @brief Compute cable lengths for a given pose
         * @param pose Platform pose
         * @return Vector of cable lengths
         */
        std::vector<double> computeCableLengths(const Pose &pose) const;

        RobotConfiguration config_; // Robot configuration
        int max_iterations_;        // Maximum iterations for optimization
        double tolerance_;          // Convergence tolerance
        double last_error_;         // Last computation error

        // Constants for optimization
        static constexpr int DEFAULT_MAX_ITERATIONS = 100;
        static constexpr double DEFAULT_TOLERANCE = 1e-6;
    };

} // namespace cable_robot_core

#endif // CABLE_ROBOT_CORE_FORWARD_KINEMATICS_HPP_