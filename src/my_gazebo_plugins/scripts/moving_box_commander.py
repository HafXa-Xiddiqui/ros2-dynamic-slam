#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovingBoxAndCylinderCommander(Node):
    def __init__(self):
        super().__init__('moving_box_and_cylinder_commander')

        # Publisher for box cmd_vel
        self.box_pub = self.create_publisher(Twist, '/moving_box/cmd_vel', 10)

        # Publisher for cylinder cmd_vel
        self.cyl_pub = self.create_publisher(Twist, '/moving_cylinder/cmd_vel', 10)

        self.timer = self.create_timer(0.1, self.command_loop)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

    def command_loop(self):
        now = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time

        # Command for box
        box_msg = Twist()
        if now < 10:
            box_msg.linear.y = 0.5  # box moves forward
        elif now < 20:
            box_msg.linear.y = -0.5  # box moves backward
        else:
            box_msg.linear.y = 0.0  # box stops

        self.box_pub.publish(box_msg)

        # Command for cylinder (example: move side to side)
        cyl_msg = Twist()
        if now < 10:
            cyl_msg.linear.y = 0.5  # cylinder moves right
        elif now < 20:
            cyl_msg.linear.y = -0.5  # cylinder moves left
        else:
            cyl_msg.linear.y = 0.0  # cylinder stops

        self.cyl_pub.publish(cyl_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MovingBoxAndCylinderCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
