#!/usr/bin/env python3
# Node Name: Controller_random
# Description:
# This ROS2 node controls a robot operating within a boundary area. It uses distance data from the boundary 
# wire sensors (`BoundaryDistances`), combined with odometry data, to adjust the robot's motion dynamically. 
# The robot either moves forward, backs off, or rotates randomly when approaching the boundary. The node 
# incorporates a backoff strategy with randomized rotation to cover the area.
#
# Key functionalities:
# - Subscribes to the `boundary_wire_distances` topic to receive distance signals from boundary wire sensors.
# - Subscribes to the `/odometry/filtered` topic to get the robot's current position and orientation.
# - Implements a randomized backoff strategy when the robot is near the boundary.
# - Publishes velocity commands to the `/cmd_vel` topic for controlling the robot's movement.
#
# Published Topic:
# - `/cmd_vel` (geometry_msgs/Twist): Velocity commands for linear and angular motion.
#
# Subscribed Topics:
# - `boundary_wire_distances` (lawn_robot_project/BoundaryDistances): Signal strengths from the boundary sensors (left, middle, right).
# - `/odometry/filtered` (nav_msgs/Odometry): Provides the robot's current position and orientation.
#
# Behavior Logic:
# - If any sensor signal exceeds a predefined safe distance, the robot backs off and rotates a random angle.
# - If all sensor signals are below the safe distance, the robot moves forward.
# - The backoff strategy stops once the randomized rotation completes, resuming normal movement.
#
# Additional Notes:
# - The randomized rotation angle is between 135° and 215°.
# - The `safe_distance` parameter defines the threshold for detecting proximity to the boundary.
# - Uses odometry data to track and control the robot's yaw (orientation) during backoff rotations.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from lawn_robot_project.msg import BoundaryDistances
from nav_msgs.msg import Odometry
import math
import random



class Controller_random(Node):
    def __init__(self):
        super().__init__('lawn_robot_control')

        # Subscribe to the boundary distances topic
        self.subscription = self.create_subscription(
            BoundaryDistances,
            'boundary_wire_distances',
            self.boundary_distances_callback,
            10
        )

        # Subscribe to the Odometry topic to get the robot's position and orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.odom_callback,
            10
        )

        # Publisher for velocity commands
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        # Variable threshold for detecting boundary wire
        self.safe_distance = 0.98

        # Variable to track the robot's current state
        self.backing_off = False
        self.current_yaw = 0.0  # Initialize yaw (robot's orientation)

    def boundary_distances_callback(self, msg):
        # Get the sensor readings
        left_signal = msg.signal_left
        middle_signal = msg.signal_middle
        right_signal = msg.signal_right

        # Create a Twist message for velocity commands
        twist = Twist()

        if not self.backing_off:
            # Determine robot's behavior based on the signal strength
            if left_signal > self.safe_distance or middle_signal > self.safe_distance or right_signal > self.safe_distance:
                # Robot is too close to the boundary, move away and initiate backoff rotation
                self.get_logger().info(f"Starting back-off")
                # Start backoff strategy by rotating a random amount between 135° and 215°
                self.backing_off = True
                self.initiate_random_rotation()


            elif left_signal < self.safe_distance and middle_signal < self.safe_distance and right_signal < self.safe_distance:
                # Robot is too far from the boundary, move towards it
                twist.linear.x=0.50

        else:
            # If robot is backing off, stop linear motion and just rotate
            twist.linear.x = 0.0
            twist.angular.z = 0.50


        # Publish the velocity command
        self.velocity_publisher.publish(twist)

        # Log the action

    def initiate_random_rotation(self):
        # Generate a random rotation angle between 135° and 215° (in degrees)
        min_rotation_deg = 135  # Minimum rotation in degrees
        max_rotation_deg = 215  # Maximum rotation in degrees
        random_rotation_deg = random.uniform(min_rotation_deg, max_rotation_deg)

        # Convert the random rotation angle to radians
        random_rotation_rad = random_rotation_deg * math.pi / 180

        # Normalize the current yaw to be within the range 0 to 2π
        normalized_current_yaw = self.normalize_angle(self.current_yaw)

        # Calculate the target rotation angle and ensure it's within the range 0 to 2π
        self.target_rotation = normalized_current_yaw + random_rotation_rad

        # Normalize the target rotation angle to ensure it's within the range 0 to 2π
        self.target_rotation = self.normalize_angle(self.target_rotation)

        # Log the rotation details
        self.get_logger().info(f"Initiated random rotation: {random_rotation_deg}° ({random_rotation_rad} radians)")
        self.get_logger().info(f"Target rotation angle (normalized to 0-2π): {self.target_rotation} radians")

    def normalize_angle(self, angle):
        # Normalize the angle to be within the range -π to +π
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def check_rotation(self, target_rotation_rad):
        # Check if the robot has rotated enough using the odom data
        #self.get_logger().info(f"{self.current_yaw}")
        current_rotation_rad = self.current_yaw  # The robot's current orientation (yaw)
        # Calculate the rotation difference and ensure it's within the target range
        rotation_diff = abs(current_rotation_rad - target_rotation_rad)  # Get the absolute rotation (in radians)
        
        if rotation_diff <= 0.1:
            self.stop_rotation()

    def stop_rotation(self):
        # Stop the rotation by setting angular velocity to zero
        twist = Twist()
        twist.angular.z = 0.0
        self.velocity_publisher.publish(twist)
        self.get_logger().info("Rotation complete. Stopping robot.")
        self.backing_off = False  # Reset backoff flag

    def odom_callback(self, msg):
        # Extract the robot's current yaw from the Odometry message
        quaternion = msg.pose.pose.orientation
        # self.get_logger().info(f"{msg.pose.pose.orientation.x},{msg.pose.pose.orientation.y},{msg.pose.pose.orientation.z},{msg.pose.pose.orientation.w}")
        _, _, self.current_yaw = self.quaternion_to_euler(quaternion)
        if self.backing_off:
            self.check_rotation(self.target_rotation)

    def quaternion_to_euler(self, quaternion):
        # Convert quaternion to Euler angles (roll, pitch, yaw)
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll =math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = math.asin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw


def main(args=None):
    rclpy.init(args=args)
    control_node = Controller_random()
    rclpy.spin(control_node)
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
