#!/usr/bin/env python3
# Node Name: BoundaryWireSensor
# Description: 
# This ROS2 node simulates a boundary wire sensor for a robot operating in a bounded area. The node computes 
# the distances of three sensors (left, middle, and right) to a predefined boundary, calculates signal 
# strengths based on these distances, and determines whether the sensors are inside or outside the boundary. 
# It publishes this information as a custom message (`BoundaryDistances`) to the topic `boundary_wire_distances`.
#
# Key functionalities:
# - Subscribes to `/robot_poses` to get the current robot pose and compute the world positions of the sensors.
# - Loads the boundary points from a YAML file (`boundary.yaml`).
# - Computes distances from sensors to the boundary using geometric calculations.
# - Adds noise to signal strengths and determines whether the sensors are inside or outside the boundary.
# - Publishes signal strengths and sensor states to the `boundary_wire_distances` topic.
#
# Published Topic:
# - `boundary_wire_distances` (lawn_robot_project/BoundaryDistances): Contains signal strengths for left, middle, and right sensors.
#
# Subscribed Topic:
# - `/robot_poses` (geometry_msgs/PoseArray): Provides the robot's pose array for computing sensor positions.
#
# Additional Notes:
# - Boundary points are loaded from a YAML file located in the `parameters` directory of the `lawn_robot_project` package.
# - Noise can be configured using the `noise_stddev` parameter for simulating real-world imperfections.
# - Signal strengths are calculated using an exponential decay function, which can be adjusted with the `signal_decay_constant` parameter.
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from lawn_robot_project.msg import BoundaryDistances
import yaml
import math
from ament_index_python.packages import get_package_share_directory
import os
import random 

def point_to_line_distance(px, py, ax, ay, bx, by):
    ABx = bx - ax
    ABy = by - ay
    APx = px - ax
    APy = py - ay
    ab_dot_ab = ABx * ABx + ABy * ABy
    if ab_dot_ab == 0:
        return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)
    t = max(0, min(1, (APx * ABx + APy * ABy) / ab_dot_ab))
    closest_x = ax + t * ABx
    closest_y = ay + t * ABy
    return math.sqrt((px - closest_x) ** 2 + (py - closest_y) ** 2)


class BoundaryWireSensor(Node):
    def __init__(self):
        super().__init__('boundary_wire_sensor')

        # Sensor offsets relative to base_link
        self.sensor_offset_left = [0.18, 0.08]
        self.sensor_offset_right = [0.18, -0.08]

        # Custom message publisher
        self.distance_publisher = self.create_publisher(BoundaryDistances, 'boundary_wire_distances', 10)

        # Subscribe to the /robot_poses topic
        self.pose_subscription = self.create_subscription(
            PoseArray,
            '/robot_poses',
            self.robot_pose_callback,
            10
        )

        # Load the boundary points from the YAML file
        self.boundary_points = self.load_boundary_from_yaml('boundary.yaml')

        # Initialize robot pose
        self.robot_pose = None

        # Signal strength decay constant (2 meters)
        self.signal_decay_constant = 2.0

        # Noise standard deviation for Gaussian noise (adjust this value as needed)
        self.noise_stddev = 0.01  # 1% noise

    def load_boundary_from_yaml(self, yaml_file):
        package_share_dir = get_package_share_directory('lawn_robot_project')
        yaml_path = os.path.join(package_share_dir, 'parameters', yaml_file)
        with open(yaml_path, 'r') as file:
            data = yaml.safe_load(file)
            return data['boundary']

    def robot_pose_callback(self, msg):
        try:
            self.robot_pose = msg.poses[4]
            self.check_boundary_wire_distance()
        except IndexError:
            self.get_logger().error("Pose array does not contain enough entries!")

    def check_boundary_wire_distance(self):
        try:
            # Robot pose
            robot_x = self.robot_pose.position.x
            robot_y = self.robot_pose.position.y
            robot_yaw = self.get_yaw_from_pose(self.robot_pose)

            # Sensor positions in the world frame
            left_sensor_world = self.transform_offset_to_world(self.sensor_offset_left, robot_x, robot_y, robot_yaw)
            right_sensor_world = self.transform_offset_to_world(self.sensor_offset_right, robot_x, robot_y, robot_yaw)
            middle_sensor_world = self.calculate_middle_sensor(left_sensor_world, right_sensor_world)

            # Calculate distances
            distance_left = self.calculate_min_distance(left_sensor_world)
            distance_right = self.calculate_min_distance(right_sensor_world)
            distance_middle = self.calculate_min_distance(middle_sensor_world)

            # Calculate signal strength
            signal_left = self.calculate_signal_strength(distance_left)
            signal_right = self.calculate_signal_strength(distance_right)
            signal_middle = self.calculate_signal_strength(distance_middle)

            # Add Gaussian noise to the signal strengths
            signal_left = self.add_gaussian_noise(signal_left)
            signal_right = self.add_gaussian_noise(signal_right)
            signal_middle = self.add_gaussian_noise(signal_middle)

            # Check if sensors are inside or outside the boundary
            inside_left = self.is_inside_boundary(left_sensor_world)
            inside_right = self.is_inside_boundary(right_sensor_world)
            inside_middle = self.is_inside_boundary(middle_sensor_world)

            # If outside, make signal strength negative
            if not inside_left:
                signal_left = -signal_left
            if not inside_right:
                signal_right = -signal_right
            if not inside_middle:
                signal_middle = -signal_middle

            # Publish custom message
            distances_msg = BoundaryDistances()
            distances_msg.header.stamp = self.get_clock().now().to_msg()
            distances_msg.header.frame_id = 'base_link'
            distances_msg.signal_left = signal_left
            distances_msg.signal_middle = signal_middle
            distances_msg.signal_right = signal_right

            self.distance_publisher.publish(distances_msg)
            #self.get_logger().info(f"Published signal strengths: Left = {signal_left}, Middle = {signal_middle}, Right = {signal_right}")

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")

    def get_yaw_from_pose(self, pose):
        q = pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y**2 + q.z**2)
        return math.atan2(siny_cosp, cosy_cosp)

    def transform_offset_to_world(self, offset, robot_x, robot_y, robot_yaw):
        offset_x, offset_y = offset
        rotated_x = offset_x * math.cos(robot_yaw) - offset_y * math.sin(robot_yaw)
        rotated_y = offset_x * math.sin(robot_yaw) + offset_y * math.cos(robot_yaw)
        world_x = robot_x + rotated_x
        world_y = robot_y + rotated_y
        return (world_x, world_y)

    def calculate_middle_sensor(self, left, right):
        return ((left[0] + right[0]) / 2, (left[1] + right[1]) / 2)

    def calculate_min_distance(self, sensor_world):
        sensor_x, sensor_y = sensor_world
        min_distance = float('inf')
        for i in range(len(self.boundary_points) - 1):
            ax, ay = self.boundary_points[i]
            bx, by = self.boundary_points[i + 1]
            distance = point_to_line_distance(sensor_x, sensor_y, ax, ay, bx, by)
            min_distance = min(min_distance, distance)
        return min_distance

    def is_inside_boundary(self, sensor_world):
        sensor_x, sensor_y = sensor_world
        # Check if the point is inside or outside based on the boundary
        total_angle = 0
        for i in range(len(self.boundary_points) - 1):
            ax, ay = self.boundary_points[i]
            bx, by = self.boundary_points[i + 1]
            angle = self.calculate_angle_to_boundary(sensor_x, sensor_y, ax, ay, bx, by)
            total_angle += angle
        # If the total angle is non-zero, the point is inside the boundary
        return abs(total_angle) > math.pi

    def calculate_angle_to_boundary(self, px, py, ax, ay, bx, by):
        # Calculate the angle between the sensor and the boundary segment
        # Returns positive if the sensor is to the left of the line, negative if to the right
        dx1, dy1 = ax - px, ay - py
        dx2, dy2 = bx - px, by - py
        cross_product = dx1 * dy2 - dy1 * dx2
        return math.atan2(cross_product, dx1 * dx2 + dy1 * dy2)

    def calculate_signal_strength(self, distance):
        # Calculate signal strength using exponential decay function
        return math.exp(-distance / self.signal_decay_constant)

    def add_gaussian_noise(self, value):
        # Add Gaussian noise to the value
        noise = random.gauss(0, self.noise_stddev)
        return value + noise


def main(args=None):
    rclpy.init(args=args)
    boundary_sensor = BoundaryWireSensor()
    rclpy.spin(boundary_sensor)
    boundary_sensor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
