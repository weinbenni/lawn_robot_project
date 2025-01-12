#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Range
from std_msgs.msg import String
import math

class LaserScanToRange(Node):
    def __init__(self):
        super().__init__('laser_scan_to_range')

        # Subscribe to the left ROS LaserScan topic
        self.laser_subscriber_left = self.create_subscription(
            LaserScan,
            '/ultrasonic_sensors/scan_left',  # ROS LaserScan topic for the left sensor
            self.laser_scan_left_callback,
            10
        )
        
        # Subscribe to the right ROS LaserScan topic
        self.laser_subscriber_right = self.create_subscription(
            LaserScan,
            '/ultrasonic_sensors/scan_right',  # ROS LaserScan topic for the right sensor
            self.laser_scan_right_callback,
            10
        )
        
        # Publisher for the left Range message
        self.range_publisher_left = self.create_publisher(Range, '/ultrasonic_sensors/range_left', 10)
        
        # Publisher for the right Range message
        self.range_publisher_right = self.create_publisher(Range, '/ultrasonic_sensors/range_right', 10)

        # Publisher for string messages (logging info to a topic)
        self.string_publisher = self.create_publisher(String, '/ultrasonic_sensors/callback_info', 10)

    def laser_scan_left_callback(self, msg: LaserScan):
        # self.get_logger().info("Got left laser scan")
        
        # Create a new Range message for the left sensor
        range_msg_left = Range()
        range_msg_left.header = msg.header
        
        # Filter out 'inf' or 'NaN' values from the scan and select the minimum range
        valid_ranges_left = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        
        if valid_ranges_left:
            range_msg_left.range = min(valid_ranges_left)  # Use the closest range measurement
            # self.get_logger().info(f"Left Range: {range_msg_left.range} meters")
        else:
            range_msg_left.range = msg.range_max  # Set to max range if no valid data
            self.get_logger().info("No valid range data from left sensor, using max range")

        # Set the field of view, max, and min range values
        range_msg_left.field_of_view = msg.angle_max - msg.angle_min
        range_msg_left.max_range = msg.range_max
        range_msg_left.min_range = msg.range_min
        

        # self.get_logger().info(f"Publishing to range_left: header={range_msg_left.header}, range={range_msg_left.range}, "
        #         f"field_of_view={range_msg_left.field_of_view}, min_range={range_msg_left.min_range}, "
        #         f"max_range={range_msg_left.max_range}")
        

        # Publish the Range message for the left sensor
        self.range_publisher_left.publish(range_msg_left)
        # self.get_logger().info("Left range message published")
        
        # Publish a string message to indicate this callback was processed
        string_msg = String()
        string_msg.data = f"Processed left laser scan with range: {range_msg_left.range:.2f} meters"
        self.string_publisher.publish(string_msg)

    def laser_scan_right_callback(self, msg: LaserScan):
        # self.get_logger().info("Got right laser scan")
        
        # Create a new Range message for the right sensor
        range_msg_right = Range()
        range_msg_right.header = msg.header
        
        # Filter out 'inf' or 'NaN' values from the scan and select the minimum range
        valid_ranges_right = [r for r in msg.ranges if not math.isinf(r) and not math.isnan(r)]
        
        if valid_ranges_right:
            range_msg_right.range = min(valid_ranges_right)  # Use the closest range measurement
            # self.get_logger().info(f"Right Range: {range_msg_right.range} meters")
        else:
            range_msg_right.range = msg.range_max  # Set to max range if no valid data
            self.get_logger().info("No valid range data from right sensor, using max range")

        # Set the field of view, max, and min range values
        range_msg_right.field_of_view = msg.angle_max - msg.angle_min
        range_msg_right.max_range = msg.range_max
        range_msg_right.min_range = msg.range_min
        # Publish the Range message for the right sensor
        self.range_publisher_right.publish(range_msg_right)



        # Publish a string message to indicate this callback was processed
        string_msg = String()
        string_msg.data = f"Processed right laser scan with range: {range_msg_right.range:.2f} meters"
        self.string_publisher.publish(string_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LaserScanToRange()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
