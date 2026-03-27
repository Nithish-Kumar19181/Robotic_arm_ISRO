#!/usr/bin/env python3
"""Publish a smooth demo trajectory to /mixer/sun_joint_cmd for visible motion in Gazebo."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SunJointDemoTrajectory(Node):
    def __init__(self):
        super().__init__('sun_joint_demo_trajectory')

        self.declare_parameter('amplitude', 1.5)
        self.declare_parameter('frequency', 0.25)
        self.declare_parameter('offset', 0.0)

        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.frequency = self.get_parameter('frequency').get_parameter_value().double_value
        self.offset = self.get_parameter('offset').get_parameter_value().double_value

        self.pub = self.create_publisher(Float64, '/mixer/sun_joint_cmd', 10)
        self.start_time = self.get_clock().now()
        self.create_timer(0.02, self._tick)

        self.get_logger().info(
            f'Demo trajectory active: amplitude={self.amplitude:.3f}, '
            f'frequency={self.frequency:.3f} Hz, offset={self.offset:.3f}'
        )

    def _tick(self):
        now = self.get_clock().now()
        t = (now - self.start_time).nanoseconds * 1e-9
        angle = self.offset + self.amplitude * math.sin(2.0 * math.pi * self.frequency * t)

        msg = Float64()
        msg.data = float(angle)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SunJointDemoTrajectory()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
