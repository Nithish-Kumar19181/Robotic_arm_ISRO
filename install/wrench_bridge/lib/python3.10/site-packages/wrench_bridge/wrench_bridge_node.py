#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped

class ForceTorqueListener(Node):
    def __init__(self):
        super().__init__('force_torque_listener')
        self.declare_parameter('topic', '/ur/force_torque')
        topic = self.get_parameter('topic').get_parameter_value().string_value
        # Use default QoS (usually fine for sensor)
        self.sub = self.create_subscription(WrenchStamped, topic, self.cb_wrench, 10)
        self.get_logger().info(f'Subscribed to: {topic}')

    def cb_wrench(self, msg: WrenchStamped):
        f = msg.wrench.force
        t = msg.wrench.torque
        stamp = msg.header.stamp
        self.get_logger().info(
            f"[{stamp.sec}.{stamp.nanosec:09d}] force=({f.x:.4f},{f.y:.4f},{f.z:.4f}) "
            f"torque=({t.x:.4f},{t.y:.4f},{t.z:.4f}) frame_id={msg.header.frame_id}"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ForceTorqueListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()