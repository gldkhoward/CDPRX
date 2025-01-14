#cable_robot_ws/src/cable_robot_core/src/lib/test_controller.py
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np

class CDPRController(Node):
    def __init__(self):
        super().__init__('cdpr_controller')
        
        # Publishers for cable lengths
        self.cable_pub = self.create_publisher(
            Float64MultiArray, 
            '/cdpr/joint_position_controller/commands', 
            10)

        # Subscribe to platform pose
        self.pose_sub = self.create_subscription(
            Pose,
            '/cdpr/platform_pose',
            self.pose_callback,
            10)
        
        # Timer for publishing commands
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Initial platform position
        self.current_pose = Pose()
        self.target_pose = Pose()
        self.target_pose.position.z = 1.0  # Start at 1m height

    def pose_callback(self, msg):
        self.current_pose = msg

    def calculate_cable_lengths(self, pose):
        # Simple geometric calculation of cable lengths
        # This is a basic implementation - you'll need proper kinematics
        cable_lengths = Float64MultiArray()
        
        # Frame corners (x, y, z)
        frame_points = [
            (1.0, 1.0, 2.0),   # Pillar 1
            (1.0, -1.0, 2.0),  # Pillar 2
            (-1.0, 1.0, 2.0),  # Pillar 3
            (-1.0, -1.0, 2.0)  # Pillar 4
        ]
        
        # Platform attachment points relative to platform center
        platform_points = [
            (0.2, 0.2, 0),   # Corner 1
            (0.2, -0.2, 0),  # Corner 2
            (-0.2, 0.2, 0),  # Corner 3
            (-0.2, -0.2, 0)  # Corner 4
        ]

        lengths = []
        for frame_p, plat_p in zip(frame_points, platform_points):
            # Transform platform point to world coordinates
            world_p = (
                pose.position.x + plat_p[0],
                pose.position.y + plat_p[1],
                pose.position.z + plat_p[2]
            )
            
            # Calculate cable length
            length = np.sqrt(
                (frame_p[0] - world_p[0])**2 +
                (frame_p[1] - world_p[1])**2 +
                (frame_p[2] - world_p[2])**2
            )
            lengths.append(length)

        cable_lengths.data = lengths
        return cable_lengths

    def control_loop(self):
        # Calculate desired cable lengths
        cable_lengths = self.calculate_cable_lengths(self.target_pose)
        
        # Publish commands
        self.cable_pub.publish(cable_lengths)

def main(args=None):
    rclpy.init(args=args)
    controller = CDPRController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()