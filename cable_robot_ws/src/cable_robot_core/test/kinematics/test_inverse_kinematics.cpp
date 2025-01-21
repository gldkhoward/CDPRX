#include <gtest/gtest.h>
#include "../../include/cable_robot_core/kinematics/inverse_kinematics.hpp"
#include "../../include/cable_robot_core/utils/utils.hpp"

using namespace cable_robot_core;

class InverseKinematicsTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        // Create a basic valid configuration for a 4-cable robot
        config_.robot_name = "test_robot";
        config_.num_cables = 4;

        // Set up platform parameters
        config_.platform.mass = 10.0;
        config_.platform.com_offset = Vector3::Zero();
        config_.platform.inertia = Matrix3::Identity();
        config_.platform.attachment_points = {
            Point3D(0.2, 0.2, 0.0),
            Point3D(-0.2, 0.2, 0.0),
            Point3D(-0.2, -0.2, 0.0),
            Point3D(0.2, -0.2, 0.0)};

        // Set up frame parameters with two pillars
        PillarParameters pillar1, pillar2;
        pillar1.mass = 50.0;
        pillar1.dimensions = Vector3(0.1, 0.1, 2.0);
        pillar1.attachment_points = {
            Point3D(1.0, 1.0, 2.0),
            Point3D(-1.0, 1.0, 2.0)};

        pillar2.mass = 50.0;
        pillar2.dimensions = Vector3(0.1, 0.1, 2.0);
        pillar2.attachment_points = {
            Point3D(-1.0, -1.0, 2.0),
            Point3D(1.0, -1.0, 2.0)};

        config_.frame.pillars = {pillar1, pillar2};
        config_.frame.dimensions = Vector3(2.0, 2.0, 2.0);

        // Set up cable parameters
        CableParameters cable_params;
        cable_params.min_length = 0.1;
        cable_params.max_length = 3.0;
        cable_params.min_tension = 10.0;
        cable_params.max_tension = 1000.0;
        cable_params.stiffness = 1000.0;
        cable_params.damping = 10.0;
        cable_params.diameter = 0.002;

        config_.cables = std::vector<CableParameters>(4, cable_params);

        // Set up workspace limits
        config_.workspace_limits.min_x = -0.5;
        config_.workspace_limits.max_x = 0.5;
        config_.workspace_limits.min_y = -0.5;
        config_.workspace_limits.max_y = 0.5;
        config_.workspace_limits.min_z = 0.5;
        config_.workspace_limits.max_z = 1.5;
        config_.workspace_limits.min_roll = -M_PI / 4;
        config_.workspace_limits.max_roll = M_PI / 4;
        config_.workspace_limits.min_pitch = -M_PI / 4;
        config_.workspace_limits.max_pitch = M_PI / 4;
        config_.workspace_limits.min_yaw = -M_PI / 4;
        config_.workspace_limits.max_yaw = M_PI / 4;
    }

    RobotConfiguration config_;
};

// Test constructor with valid configuration
TEST_F(InverseKinematicsTest, ConstructorValidConfig)
{
    EXPECT_NO_THROW(InverseKinematics ik(config_));
}

// Test constructor with invalid number of cables
TEST_F(InverseKinematicsTest, ConstructorInvalidNumCables)
{
    config_.num_cables = 0;
    EXPECT_THROW(InverseKinematics ik(config_), std::invalid_argument);
}

// Test constructor with mismatched platform attachment points
TEST_F(InverseKinematicsTest, ConstructorMismatchedAttachmentPoints)
{
    config_.platform.attachment_points.pop_back();
    EXPECT_THROW(InverseKinematics ik(config_), std::invalid_argument);
}

// Test constructor with invalid cable parameters
TEST_F(InverseKinematicsTest, ConstructorInvalidCableParams)
{
    config_.cables[0].min_length = -1.0;
    EXPECT_THROW(InverseKinematics ik(config_), std::invalid_argument);

    config_.cables[0].min_length = 0.1;
    config_.cables[0].max_length = 0.05;
    EXPECT_THROW(InverseKinematics ik(config_), std::invalid_argument);

    config_.cables[0].min_tension = -10.0;
    EXPECT_THROW(InverseKinematics ik(config_), std::invalid_argument);
}

// Test calculation of cable lengths with valid pose
TEST_F(InverseKinematicsTest, CalculateCableLengthsValidPose)
{
    InverseKinematics ik(config_);

    Pose pose;
    pose.position = Vector3(0.0, 0.0, 1.0);
    pose.orientation = Matrix3::Identity();

    std::vector<double> lengths;
    EXPECT_NO_THROW(lengths = ik.calculateCableLengths(pose));
    EXPECT_EQ(lengths.size(), config_.num_cables);

    // Verify lengths are within limits
    for (size_t i = 0; i < lengths.size(); ++i)
    {
        EXPECT_GE(lengths[i], config_.cables[i].min_length);
        EXPECT_LE(lengths[i], config_.cables[i].max_length);
    }
}

// Test calculation with pose outside workspace position limits
TEST_F(InverseKinematicsTest, CalculateCableLengthsInvalidPosition)
{
    InverseKinematics ik(config_);

    Pose pose;
    pose.position = Vector3(1.0, 0.0, 1.0); // x outside workspace
    pose.orientation = Matrix3::Identity();

    EXPECT_THROW(ik.calculateCableLengths(pose), std::invalid_argument);
}

// Test calculation with pose outside workspace orientation limits
// TODO: Impliment and Uncomments
// TEST_F(InverseKinematicsTest, CalculateCableLengthsInvalidOrientation)
// {
//     InverseKinematics ik(config_);

//     Pose pose;
//     pose.position = Vector3(0.0, 0.0, 1.0);

//     // Create rotation matrix for roll angle outside limits
//     double roll = M_PI / 2; // Outside ±π/4 limits
//     pose.orientation = Eigen::AngleAxisd(roll, Vector3::UnitX()).matrix();

//     EXPECT_THROW(ik.calculateCableLengths(pose), std::invalid_argument);
// }

// Test calculation with pose that would result in cable lengths outside limits
TEST_F(InverseKinematicsTest, CalculateCableLengthsOutsideLimits)
{
    InverseKinematics ik(config_);

    Pose pose;
    pose.position = Vector3(0.0, 0.0, 0.05); // Very low z position
    pose.orientation = Matrix3::Identity();

    EXPECT_THROW(ik.calculateCableLengths(pose), std::invalid_argument);
}

// Test symmetry of cable lengths for symmetric poses
TEST_F(InverseKinematicsTest, CableLengthsSymmetry)
{
    InverseKinematics ik(config_);

    Pose pose1, pose2;
    pose1.position = Vector3(0.2, 0.0, 1.0);
    pose2.position = Vector3(-0.2, 0.0, 1.0);
    pose1.orientation = pose2.orientation = Matrix3::Identity();

    auto lengths1 = ik.calculateCableLengths(pose1);
    auto lengths2 = ik.calculateCableLengths(pose2);

    // For symmetric configuration, expect symmetric cable lengths
    EXPECT_NEAR(lengths1[0], lengths2[1], 1e-10);
    EXPECT_NEAR(lengths1[3], lengths2[2], 1e-10);
}

int main(int argc, char **argv)
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}