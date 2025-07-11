#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class MovingBoxAndCylinderCommander(Node):
    def __init__(self):
        super().__init__('moving_box_and_cylinder_commander')

        self.box_pub = self.create_publisher(Twist, '/moving_box/cmd_vel', 10)
        self.cyl_pub = self.create_publisher(Twist, '/moving_cylinder/cmd_vel', 10)

        self.timer_period = 0.05                     # 20 Hz update
        self.timer = self.create_timer(self.timer_period, self.command_loop)
        self.start_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Motion parameters
        self.box_amp  = 0.5      # m/s peak forward/back
        self.cyl_amp  = 0.5      # m/s peak side‑to‑side
        self.freq     = 0.04      # Hz → one full cycle every 10 s (ω = 2πf)

    def command_loop(self):
        t = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        omega = 2 * math.pi * self.freq

        # ---------- box: forward/back along x ----------
        box_twist = Twist()
        box_twist.linear.x = self.box_amp * math.sin(omega * t)
        self.box_pub.publish(box_twist)

        # ---------- cylinder: side‑to‑side along y ----------
        cyl_twist = Twist()
        cyl_twist.linear.y = self.cyl_amp * math.sin(omega * t + math.pi/2)  # phase‑shift if desired
        self.cyl_pub.publish(cyl_twist)

def main(args=None):
    rclpy.init(args=args)
    node = MovingBoxAndCylinderCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
