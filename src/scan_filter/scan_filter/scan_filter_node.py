#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from builtin_interfaces.msg import Time
from rclpy.time import Time as RclTime

class ScanFilter(Node):
    def __init__(self):
        super().__init__('scan_filter')
        self.sub_scan = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.sub_detections = self.create_subscription(String, '/dynamic_objects/bboxes', self.detections_callback, 10)
        self.pub_filtered = self.create_publisher(LaserScan, '/scan_filtered', 10)
        
        self.last_detection_time = None

    def detections_callback(self, msg):
        # Assume msg.data is a stringified list of detected objects
        if msg.data and msg.data != "[]":
            self.last_detection_time = self.get_clock().now()

    def scan_callback(self, msg):
        now = self.get_clock().now()

        # Copy original scan
        filtered = LaserScan()
        filtered = msg
        filtered.ranges = list(msg.ranges)

        # If we saw a detection within last 2 seconds, mask center area
        if self.last_detection_time and (now - self.last_detection_time).nanoseconds < 2e9:
            self.get_logger().info("Masking scan due to recent detection")
            angle_min = msg.angle_min
            angle_increment = msg.angle_increment
            for i in range(len(msg.ranges)):
                angle = angle_min + i * angle_increment
                if -0.26 < angle < 0.26:  # ~30 degrees in front
                    filtered.ranges[i] = float('inf')

        self.pub_filtered.publish(filtered)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
