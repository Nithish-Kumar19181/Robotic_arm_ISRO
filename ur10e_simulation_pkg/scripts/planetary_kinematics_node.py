#!/usr/bin/env python3
"""
planetary_kinematics_node.py  — ROS 2 (Humble)
────────────────────────────────────────────────
MY-300 planetary mixer kinematic constraints.

  S = 4 (sun teeth)   P = 2 (planet teeth)   R = S + 2P = 8 (ring teeth)

  θ_shroud = θ_sun / 3                      (same direction as sun)
  θ_orbit  = θ_sun / 3                      (same as shroud)
  θ_spin   = -(R/P) × θ_shroud = -4/3 × θ_sun  (opposite to sun)

Subscribes:
  /joint_states_gui     — sun_joint position from joint_state_publisher_gui
  /sun_joint_cmd        — Float64 direct command (optional)

Publishes:
  /joint_states         — all 6 joints → robot_state_publisher → RViz TF
  /forward_position_controller/commands  — ros2_control ForwardCommandController
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_msgs.msg import Float64MultiArray

# ── Gear constants ─────────────────────────────────────────────────────────
T_SUN = 4
T_PLANET = 2
T_RING = T_SUN + 2 * T_PLANET  # 8

SHROUD_MULT = 1.0 / 3.0   #  θ_sun / 3    (same dir as sun)
ORBIT_MULT = 1.0 / 3.0    #  θ_sun / 3    (same as shroud)
SPIN_MULT = -4.0 / 3.0    # -4/3 × θ_sun  (opposite to sun)

JOINT_NAMES = [
    'sun_joint',
    'shroud_joint',
    'planet_1_orbit_joint',
    'planet_1_spin_joint',
    'planet_2_orbit_joint',
    'planet_2_spin_joint',
]


class PlanetaryKinematicsNode(Node):

    def __init__(self):
        super().__init__('planetary_kinematics_node')
        self.theta_sun = 0.0

        # ── Publishers ──────────────────────────────────────────────────────
        # 1) /joint_states  →  robot_state_publisher  →  RViz TF
        self.joint_state_pub = self.create_publisher(
            JointState, '/joint_states', 10)

        # 2) /forward_position_controller/commands  →  ros2_control
        self.gazebo_cmd_pub = self.create_publisher(
            Float64MultiArray,
            '/forward_position_controller/commands',
            10)

        # ── Subscribers ─────────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/joint_states_gui', self._gui_cb, 10)

        self.create_subscription(
            Float64, '/sun_joint_cmd', self._cmd_cb, 10)

        # ── 50 Hz publish timer ─────────────────────────────────────────────
        self.create_timer(0.02, self._timer_cb)

        self.get_logger().info('=' * 58)
        self.get_logger().info('Planetary Mixer Kinematics  (MY-300)  — ROS 2 Humble')
        self.get_logger().info(f'  S={T_SUN}  P={T_PLANET}  R={T_RING}')
        self.get_logger().info(f'  shroud = {SHROUD_MULT:+.4f} × θ_sun')
        self.get_logger().info(f'  orbit  = {ORBIT_MULT:+.4f} × θ_sun')
        self.get_logger().info(f'  spin   = {SPIN_MULT:+.4f} × θ_sun')
        self.get_logger().info(
            'Publishes → /joint_states (RViz)'
            ' + /forward_position_controller/commands (ros2_control)')
        self.get_logger().info('=' * 58)

    # ── callbacks ──────────────────────────────────────────────────────────
    def _gui_cb(self, msg: JointState):
        if 'sun_joint' in msg.name:
            idx = msg.name.index('sun_joint')
            if idx < len(msg.position):
                self.theta_sun = msg.position[idx]

    def _cmd_cb(self, msg: Float64):
        self.theta_sun = msg.data

    # ── kinematics + publish ───────────────────────────────────────────────
    def _timer_cb(self):
        ts = self.theta_sun

        positions = [
            ts,                  # sun_joint
            SHROUD_MULT * ts,    # shroud_joint
            ORBIT_MULT * ts,     # planet_1_orbit_joint
            SPIN_MULT * ts,      # planet_1_spin_joint
            ORBIT_MULT * ts,     # planet_2_orbit_joint
            SPIN_MULT * ts,      # planet_2_spin_joint
        ]

        # --- /joint_states (RViz) -------------------------------------------
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = JOINT_NAMES
        js.position = positions
        js.velocity = [0.0] * 6
        js.effort = [0.0] * 6
        self.joint_state_pub.publish(js)

        # --- /forward_position_controller/commands (Gazebo) -----------------
        cmd = Float64MultiArray()
        cmd.data = positions   # same order as joints in controllers.yaml
        self.gazebo_cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PlanetaryKinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
