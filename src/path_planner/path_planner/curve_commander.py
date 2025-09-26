#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory
from builtin_interfaces.msg import Duration
import numpy as np

class CylinderCleaner(Node):
    def __init__(self):
        super().__init__('cylinder_cleaner')

        self._client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory')

        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        self.get_logger().info('Waiting for action server...')
        self._client.wait_for_server()
        self.get_logger().info('Action server ready.')

        # Parameters for cylindrical cleaning motion
        self.radius = 0.5      # m (distance from robot base axis)
        self.base_height = 0.3 # m (base height above ground)
        self.height_range = 0.4 # m (vertical sweep range)
        self.n_circles = 3     # number of vertical circles
        self.npts_per_circle = 40  # points per circle
        self.dt = 0.15         # time between points

        self.send_cleaning_trajectory()

    def send_cleaning_trajectory(self):
        points = []
        time = 0.0

        # Total points in trajectory
        total_points = self.n_circles * self.npts_per_circle
        
        for i in range(total_points):
            pt = JointTrajectoryPoint()
            
            # Calculate current position in spiral
            circle_num = i // self.npts_per_circle
            point_in_circle = i % self.npts_per_circle
            
            # Pan angle (full circles)
            pan_angle = 2.0 * math.pi * point_in_circle / self.npts_per_circle
            
            # Height variation (spiral up and down)
            height_factor = math.sin(2.0 * math.pi * circle_num / self.n_circles)
            current_height = self.base_height + (self.height_range / 2.0) * height_factor
            
            # Calculate joint positions using inverse kinematics for cylindrical coordinates
            joint_positions = self.calculate_ik(pan_angle, self.radius, current_height)
            
            pt.positions = joint_positions
            time += self.dt
            pt.time_from_start = Duration(sec=int(time),
                                          nanosec=int((time - int(time)) * 1e9))
            points.append(pt)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = self.joint_names
        goal_msg.trajectory.points = points

        self.get_logger().info(f'Sending cylindrical cleaning trajectory with {len(points)} points...')
        send_future = self._client.send_goal_async(goal_msg)
        send_future.add_done_callback(self.goal_response_callback)

    def calculate_ik(self, pan_angle, radius, height):
        """
        Calculate inverse kinematics for UR robot to point tool radially outward
        at given cylindrical coordinates (pan_angle, radius, height)
        """
        # Base position (x, y, z) in cylindrical coordinates
        x = radius * math.cos(pan_angle)
        y = radius * math.sin(pan_angle)
        z = height
        
        # For UR robots, we can use a simplified IK approach
        # Calculate the required joint angles to point tool radially outward
        
        # Joint 0: Shoulder pan - points directly at the target angle
        joint0 = pan_angle  # This makes the arm point in the right direction
        
        # Calculate distance from base to target point in horizontal plane
        horizontal_distance = math.sqrt(x**2 + y**2)
        
        # For a UR10e-like robot, approximate IK for reaching the point
        # This is a simplified calculation - you may need to adjust based on your specific robot
        
        # Target distance from shoulder to wrist (simplified)
        target_distance = math.sqrt(horizontal_distance**2 + (z - 0.16)**2)  # 0.16 is approximate shoulder height
        
        # Using law of cosines for shoulder and elbow joints
        # UR10e link lengths (adjust for your specific robot)
        a1 = 0.18   # shoulder to elbow
        a2 = 0.48   # elbow to wrist
        
        # Calculate elbow angle
        cos_elbow = (a1**2 + a2**2 - target_distance**2) / (2 * a1 * a2)
        cos_elbow = max(min(cos_elbow, 1.0), -1.0)  # Clamp to valid range
        joint2 = math.pi - math.acos(cos_elbow)  # Elbow joint
        
        # Calculate shoulder angle
        alpha = math.atan2(z - 0.16, horizontal_distance)  # 0.16 is approximate shoulder height
        beta = math.acos((a1**2 + target_distance**2 - a2**2) / (2 * a1 * target_distance))
        joint1 = - (alpha + beta)  # Shoulder lift joint (negative for UR convention)
        
        # Wrist joints to keep tool pointing radially outward
        # Tool should point in the direction of the cylinder's radius vector
        joint3 = -joint1 - joint2  # Wrist 1 - keeps tool horizontal
        joint4 = -pan_angle        # Wrist 2 - points tool radially outward
        joint5 = 0.0               # Wrist 3 - no rotation
        
        # Apply joint limits (approximate UR10e limits)
        joint_limits = [
            (-math.pi, math.pi),      # joint0
            (-2.0 * math.pi, 0.0),    # joint1 (approx limits)
            (0.0, 2.0 * math.pi),     # joint2 (approx limits)  
            (-2.0 * math.pi, 0.0),    # joint3 (approx limits)
            (-2.0 * math.pi, 2.0 * math.pi), # joint4
            (-2.0 * math.pi, 2.0 * math.pi)  # joint5
        ]
        
        joint_positions = [joint0, joint1, joint2, joint3, joint4, joint5]
        
        # Apply limits
        for i in range(6):
            joint_positions[i] = max(joint_limits[i][0], min(joint_limits[i][1], joint_positions[i]))
        
        return joint_positions

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            rclpy.shutdown()
            return
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Trajectory execution successful!')
        else:
            self.get_logger().error(f'Trajectory execution failed with error code: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CylinderCleaner()
    rclpy.spin(node)
    node.destroy_node()

if __name__ == '__main__':
    main()