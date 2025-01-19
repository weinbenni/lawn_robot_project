#!/usr/bin/env python3
# Node Name: CoverageTracker
# Description: 
# This ROS2 node tracks the movement of a robot by subscribing to a topic (`/robot_poses`) that publishes 
# an array of poses (`PoseArray`). It extracts the 5th pose in the array (index 4), appends it to a path 
# (`Path` message), and publishes the updated path to the `/robot_path` topic. The path can be used for 
# visualization or analysis of the robot's coverage area.
#
# Key functionalities:
# - Subscribes to `/robot_poses` to receive updated pose information of the robot.
# - Extracts and tracks a specific pose (5th pose) from the array.
# - Maintains and publishes a continuous path on the `/robot_path` topic.
#
# Published Topic:
# - `/robot_path` (nav_msgs/Path): The accumulated path of the robot.
#
# Subscribed Topic:
# - `/robot_poses` (geometry_msgs/PoseArray): The array of robot poses.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class CoverageTracker(Node):
    def __init__(self):
        super().__init__('coverage_tracker')
        self.robot_path = Path()  # To store the path of the robot
        self.robot_pose = None

        # Publishers
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)

        # Subscribe to robot poses
        self.pose_subscription = self.create_subscription(
            PoseArray,
            '/robot_poses',
            self.robot_pose_callback,
            10
        )

    def robot_pose_callback(self, msg):
        try:
            # Assuming you want to track the 5th pose in the array (index 4)
            self.robot_pose = msg.poses[4]
            self.robot_path.poses.append(PoseStamped(pose=self.robot_pose))
            self.publish_robot_path()
        except IndexError:
            self.get_logger().error("Pose array does not contain enough entries!")

    def publish_robot_path(self):
        # Publish the updated path
        self.robot_path.header.stamp = self.get_clock().now().to_msg()
        self.robot_path.header.frame_id = 'odom'  # Adjust frame_id as needed
        self.path_publisher.publish(self.robot_path)

def main(args=None):
    rclpy.init(args=args)
    coverage_tracker = CoverageTracker()
    rclpy.spin(coverage_tracker)
    coverage_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
