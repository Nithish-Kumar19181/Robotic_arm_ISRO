#pragma once

#include "ur10e_force_control_base/robot_state.hpp"
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include"ur10e_force_control_base/kinematics_helper.hpp"

namespace motion_primitives 
{

class MotionPrimitiveBase {
public:
  virtual ~MotionPrimitiveBase() = default;

  virtual void onStart(RobotState &robot_state, std::string &urdf_string) = 0;

  virtual void update(RobotState &robot_state, double dt) = 0;

  virtual void targetPose(geometry_msgs::msg::PoseStamped &pose) = 0;

  virtual void targetWrench(geometry_msgs::msg::WrenchStamped &wrench) = 0;

  virtual bool isDone() const = 0;

  virtual double getProgress() const = 0;

  virtual std::string getName() const = 0;

  virtual void reset() { initialized_ = false; }

  bool isInitialized() const { return initialized_; }

private:
  bool initialized_{false};
  double progress_{0.0};
  bool finished_{false};
  double elapsed_{0.0};
};
} // namespace motion_primitives
