#!/usr/bin/env python3
# Node Name: Controller_b_pattern
# Description:
# This ROS2 node manages a garden robot designed to follow a boundary wire and operate within a designated area. 
# It uses boundary wire sensor data (`BoundaryDistances`) and odometry data to determine the robot's movement, 
# employing state-based control logic to navigate, straighten, reverse, and make precise turns. The robot's goal 
# is to efficiently cover the garden area while avoiding crossing the boundary wire.
#
# Key Functionalities:
# - Subscribes to the `boundary_wire_distances` topic to monitor sensor signals indicating proximity to the boundary wire.
# - Subscribes to the `/odometry/filtered` topic for position and orientation updates, used for yaw tracking and distance calculation.
# - Implements various states to control behavior: driving straight, straightening, reversing, and turning.
# - Maintains a safe distance (`safe_distance`) from the boundary wire and reacts to sensor signals dynamically.
# - Uses odometry to calculate the robot's travel distance and control its rotation angles for precise maneuvers.
#
# Published Topic:
# - `/cmd_vel` (geometry_msgs/Twist): Velocity commands for both linear and angular motion.
#
# Subscribed Topics:
# - `/boundary_wire_distances` (lawn_robot_project/BoundaryDistances): Sensor signals for boundary detection (left, middle, right).
# - `/odometry/filtered` (nav_msgs/Odometry): Provides the robot's current position and orientation.
#
# Behavior Logic:
# - Drives straight until the boundary wire is detected, triggering state transitions for straightening or reversing.
# - Straightens the robot based on left and right sensor signal values for boundary alignment.
# - Reverses the robot a fixed distance upon detecting the boundary and turns by a specified angle (e.g., 88 degrees).
# - Internal states (`INITIAL`, `DRIVE_STRAIGHT`, `STRAIGHTEN`, etc.) ensure robust and sequential decision-making.
#
# Additional Notes:
# - The robot alternates turn directions to avoid repetitive patterns (`last_rot_dir` toggles between 'left' and 'right').
# - Debug mode is available for detailed logging of the robot's internal state and decision-making process.
# - Configurable parameters, such as `safe_distance` and rotation angles, allow customization for different environments.

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from lawn_robot_project.msg import BoundaryDistances
from random import uniform
import math

class Controller_b_pattern(Node):
    def __init__(self):
        super().__init__('garden_robot_controller')
        
        # Robot state variables
        self.current_yaw = 0.0
        self.target_rotation = None
        self.wire_detected = False
        self.distance_traveled = 0.0
        self.boundary_signals = {'left': 0.0, 'middle': 0.0, 'right': 0.0}
        self.initial_position = None
        self.position = None
        self.target_dist = 0.0
        self.safe_distance = 0.97
        self.last_rot_dir = 'left'

        # Publishers and subscribers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.boundary_subscriber = self.create_subscription(
            BoundaryDistances, '/boundary_wire_distances', self.boundary_callback, 10
        )
        self.odom_subscriber = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10
        )

        # Timer for main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # Internal control state
        self.state = 'INITIAL'

        # Debug mode
        self.debug = False

    def log_debug(self, message):
        if self.debug:
            self.get_logger().info(f"DEBUG: {message}")

    def boundary_callback(self, msg):
        self.boundary_signals['left'] = msg.signal_left
        self.boundary_signals['middle'] = msg.signal_middle
        self.boundary_signals['right'] = msg.signal_right

        # Check if middle sensor detects the wire
        if msg.signal_left > self.safe_distance or msg.signal_middle > self.safe_distance or msg.signal_right > self.safe_distance:
            if self.state == 'DRIVE_STRAIGHT' or self.state == 'INITIAL':
                self.wire_detected = True
                self.log_debug(f"Wire detected by middle sensor. Signals - Left: {msg.signal_left}, Middle: {msg.signal_middle}, Right: {msg.signal_right}")

    def odom_callback(self, msg):
        # Update current yaw and distance traveled
        self.position = msg.pose.pose.position
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_yaw = self.quaternion_to_euler(orientation_q)
        # self.log_debug(f"Current yaw updated: {self.current_yaw}")

    def quaternion_to_euler(self, q):
        # Convert quaternion to Euler angles
        t0 = +2.0 * (q.w * q.x + q.y * q.z)
        t1 = +1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (q.w * q.y - q.z * q.x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (q.w * q.z + q.x * q.y)
        t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def control_loop(self):
        self.log_debug(f"Control loop state: {self.state}")
        if self.state == 'INITIAL':
            self.drive_straight()
        if self.state == 'DRIVE_STRAIGHT':
            self.drive_straight()
        elif self.state == 'STRAIGHTEN':
            self.straighten()
        elif self.state == 'CHECK_ROT1':
            self.check_rotation(self.target_rotation)
        elif self.state == 'REVERSE_STRAIGHT':
            self.check_reverse_straight(self.target_dist)
        elif self.state == 'CHECK_ROT2':
            self.check_rotation(self.target_rotation)

    def drive_straight(self):
        # self.log_debug("Driving straight.")
        if self.wire_detected:
            self.wire_detected = False
            self.stop_robot()
            self.get_logger().info("Wire detected. Transitioning to straightening state.")
            if self.state == 'INITIAL':
                self.state = 'STRAIGHTEN'
            else:
                self.get_logger().info(" Found boundary. Transitioning to reverse.")
                self.state = 'CHECK_ROT1'
                self.initiate_rotation(88)
                
        else:
            twist = Twist()
            twist.linear.x = 0.3  # Move straight
            self.cmd_vel_publisher.publish(twist)

    def straighten(self):
        #self.log_debug("Straightening robot based on boundary sensor values.")
        twist = Twist()

        if self.boundary_signals['left'] > self.boundary_signals['right']:
            twist.angular.z = 0.1  # Rotate slightly to the right
        elif self.boundary_signals['right'] > self.boundary_signals['left']:
            twist.angular.z = -0.1  # Rotate slightly to the left
        else:
            twist.angular.z = 0.0  # Already aligned

        self.cmd_vel_publisher.publish(twist)

        if abs(self.boundary_signals['left'] - self.boundary_signals['right']) < 0.03:
            self.get_logger().info("Straightening complete. Transitioning to reverse.")
            self.state = 'CHECK_ROT1'
            self.initiate_rotation(88)
            
            

    def initiate_rotation(self, deg):
        # Normalize the current yaw to be within the range 0 to 2π
        normalized_current_yaw = self.normalize_angle(self.current_yaw)
        if self.last_rot_dir == 'right':
            random_rotation_rad = deg * math.pi / 180
            # Calculate the target rotation angle and ensure it's within the range 0 to 2π
            self.target_rotation = normalized_current_yaw + random_rotation_rad

            # Normalize the target rotation angle to ensure it's within the range 0 to 2π
            self.target_rotation = self.normalize_angle(self.target_rotation)

            self.log_debug(f"Initiated random rotation: {deg} degrees ({random_rotation_rad} radians). Target rotation: {self.target_rotation}")
            # Give command to rotate
            twist = Twist()
            twist.angular.z = 0.50
            # Publish the velocity command
            self.cmd_vel_publisher.publish(twist)
            if self.state == 'CHECK_ROT2':
                self.last_rot_dir = 'left'
        
        elif self.last_rot_dir == 'left':
            random_rotation_rad = -deg * math.pi / 180
            # Calculate the target rotation angle and ensure it's within the range 0 to 2π
            self.target_rotation = normalized_current_yaw + random_rotation_rad

            # Normalize the target rotation angle to ensure it's within the range 0 to 2π
            self.target_rotation = self.normalize_angle(self.target_rotation)

            self.log_debug(f"Initiated random rotation: {deg} degrees ({random_rotation_rad} radians). Target rotation: {self.target_rotation}")
            # Give command to rotate
            twist = Twist()
            twist.angular.z = -0.50
            # Publish the velocity command
            self.cmd_vel_publisher.publish(twist)
            if self.state == 'CHECK_ROT2':
                self.last_rot_dir = 'right'

    def check_rotation(self, target_rotation_rad):
        # Check if the robot has rotated enough using the odom data
        #self.get_logger().info(f"{self.current_yaw}")
        current_rotation_rad = self.current_yaw  # The robot's current orientation (yaw)
        # Calculate the rotation difference and ensure it's within the target range
        rotation_diff = abs(current_rotation_rad - target_rotation_rad)  # Get the absolute rotation (in radians)
        
        if rotation_diff <= 0.05:
            twist = Twist()
            twist.angular.z = 0.0
            # Publish the velocity command
            self.cmd_vel_publisher.publish(twist)
            if self.state == 'CHECK_ROT1':
                self.initiate_straight_reverse(0.2)
                self.state = 'REVERSE_STRAIGHT'
            elif self.state == 'CHECK_ROT2':
                self.state = 'DRIVE_STRAIGHT'
        

    def initiate_straight_reverse(self, dist):
        # Normalize the target rotation angle to ensure it's within the range 0 to 2π
        self.initial_position = self.position
        self.target_dist = dist

        self.log_debug(f"Initiated straight reverse move: {dist} m.")
        # Give command to rotate
        twist = Twist()
        twist.angular.z = 0.0
        twist.linear.x = 0.50
        # Publish the velocity command
        self.cmd_vel_publisher.publish(twist)

    def check_reverse_straight(self, target_dist):
        current_position = self.position
        distance_traveled = self.calculate_distance(self.initial_position, current_position)

        self.log_debug(f"Distance travelled so far: {distance_traveled} m.")

        if distance_traveled >= target_dist:
            self.stop_robot()
            self.state = 'CHECK_ROT2'
            self.initiate_rotation(88)  # Perform 90-degree turn
            

    def stop_robot(self):
        self.log_debug("Stopping the robot.")
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist)

        self.moving_straight = False

    def normalize_angle(self,angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def calculate_distance(self, start, current):
        dx = current.x - start.x
        dy = current.y - start.y
        distance = math.sqrt(dx**2 + dy**2)
        self.log_debug(f"Calculating distance: Start=({start.x}, {start.y}), Current=({current.x}, {current.y}), Distance={distance:.3f}")
        return distance

def main(args=None):
    rclpy.init(args=args)
    controller = Controller_b_pattern()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
