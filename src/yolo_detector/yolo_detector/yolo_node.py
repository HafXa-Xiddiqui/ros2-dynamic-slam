#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import torch

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(String, '/dynamic_objects/bboxes', 10)
        self.bridge = CvBridge()
        self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model(frame)

        boxes = results.pandas().xyxy[0]
        dets = [f"{row['name']} ({int(row['confidence']*100)}%)" for idx, row in boxes.iterrows()]
        self.get_logger().info(f"Detected: {dets}")

        self.pub.publish(String(data=str(dets)))

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()