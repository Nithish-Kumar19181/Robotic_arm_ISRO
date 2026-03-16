#pragma once

#include "ur10e_force_control_base/force_controller.hpp"
#include "ur10e_force_control_base/kinematics_helper.hpp"
#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include <rclcpp/rclcpp.hpp>
using namespace kinematics_helper;

namespace motion_primitives {

class HelixPrimitive : public MotionPrimitiveBase {
public:
  HelixPrimitive() = default;

  void onStart(RobotState &robot_state, std::string &urdf_string) override;

  void update(RobotState &robot_state, double dt) override;

  void targetPose(geometry_msgs::msg::PoseStamped &pose) override;

  void targetWrench(geometry_msgs::msg::WrenchStamped &wrench) override;

  bool isDone() const override;

  double getProgress() const override;

  std::string getName() const override { return "HelixPrimitive"; }
  
  void setParameters(double angular_speed_, double max_angle_rad_,double target_height_, double direction_);

private:

  double radius_{0.0};
  double theta_{0.0};
  double traveled_angle_{0.0};
  double elapsed_{0.0};
  double start_z{0.0}, start_x{0.0}, start_y{0.0};
  double qx_{0.0}, qy_{0.0}, qz_{0.0}, qw_{1.0};

  double angular_speed_{0.25};
  double max_angle_rad_{2 * M_PI};
  double accel_ratio_{0.15};
  double target_height_{-10.0};
  int direction_{-1}; // -1 for CW, 1 for CCW

  bool initialized_{false};
  bool isDone_{false};
  
  rclcpp::Time start_time_;
  double progress_{0.0};

  KinematicsHelper kinematicsHelper_;
};

} // namespace motion_primitives
