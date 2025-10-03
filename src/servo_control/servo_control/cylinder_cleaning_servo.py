import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import math

class CylinderCleaningServo(Node):
    def __init__(self):
        super().__init__('cylinder_cleaning_servo')
        # Publish to the correct MoveIt Servo topic
        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.timer = self.create_timer(0.02, self.publish_cmd)
        self.t0 = self.get_clock().now()

    def publish_cmd(self):
        t = (self.get_clock().now() - self.t0).nanoseconds * 1e-9
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'tool0'  # express twist in end-effector frame
        # Circular XY trajectory + vertical oscillation
        radius_speed = 0.2
        freq = 0.3
        cmd.twist.linear.x = -radius_speed * math.sin(freq * t)
        cmd.twist.linear.y =  radius_speed * math.cos(freq * t)
        cmd.twist.linear.z = 0.05 * math.sin(0.15 * t)
        self.pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = CylinderCleaningServo()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
