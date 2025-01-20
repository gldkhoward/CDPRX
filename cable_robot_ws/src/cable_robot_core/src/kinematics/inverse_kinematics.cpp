#include "../../include/cable_robot_core/kinematics/inverse_kinematics.hpp"
#include "../../include/cable_robot_core/utils/utils.hpp"
#include <stdexcept>

namespace cable_robot_core
{

    InverseKinematics::InverseKinematics(const RobotConfiguration &config)
        : config_(config)
    {
        // Validate configuration
        if (config.num_cables == 0)
        {
            throw std::invalid_argument("Robot must have at least one cable");
        }

        if (config.platform.attachment_points.size() != config.num_cables)
        {
            throw std::invalid_argument("Number of platform attachment points must match number of cables");
        }

        // Calculate total number of anchor points across all pillars
        std::size_t total_anchor_points = 0;
        for (const auto &pillar : config.frame.pillars)
        {
            total_anchor_points += pillar.attachment_points.size();
        }

        if (total_anchor_points != config.num_cables)
        {
            throw std::invalid_argument("Total number of frame anchor points must match number of cables");
        }

        // Validate cable parameters
        for (const auto &cable : config.cables)
        {
            if (cable.min_length <= 0 || cable.max_length <= cable.min_length)
            {
                throw std::invalid_argument("Invalid cable length limits");
            }
            if (cable.min_tension < 0 || cable.max_tension <= cable.min_tension)
            {
                throw std::invalid_argument("Invalid cable tension limits");
            }
        }
    }

    std::vector<double> InverseKinematics::calculateCableLengths(const Pose &pose) const
    {
        // Validate pose is within workspace limits
        if (!utils::isPositionInWorkspace(pose.position, config_.workspace_limits))
        {
            throw std::invalid_argument("Platform position outside workspace limits");
        }

        if (!utils::isOrientationInWorkspace(pose.orientation, config_.workspace_limits))
        {
            throw std::invalid_argument("Platform orientation outside workspace limits");
        }

        std::vector<double> cable_lengths(config_.num_cables);
        std::size_t cable_idx = 0;

        // Iterate through all pillars to access anchor points
        for (std::size_t pillar_idx = 0; pillar_idx < config_.frame.pillars.size(); ++pillar_idx)
        {
            const auto &pillar = config_.frame.pillars[pillar_idx];

            // Process each anchor point on the current pillar
            for (std::size_t point_idx = 0; point_idx < pillar.attachment_points.size(); ++point_idx)
            {
                // Get anchor point in world frame (already in world frame as it's on the pillar)
                const auto &anchor_point = pillar.attachment_points[point_idx];

                // Transform platform attachment point from platform frame to world frame
                const auto &platform_attach_local = config_.platform.attachment_points[cable_idx];
                Point3D platform_attach_world = utils::transformToWorldFrame(platform_attach_local, pose);

                // Calculate cable vector and length
                auto [unit_vector, length] = utils::calculateCableVectorAndLength(
                    anchor_point, platform_attach_world);

                // Validate cable length
                const auto &cable_params = config_.cables[cable_idx];
                if (length < cable_params.min_length || length > cable_params.max_length)
                {
                    throw std::invalid_argument("Calculated cable length outside allowable limits");
                }

                cable_lengths[cable_idx] = length;
                cable_idx++;
            }
        }

        return cable_lengths;
    }

} // namespace cable_robot_core