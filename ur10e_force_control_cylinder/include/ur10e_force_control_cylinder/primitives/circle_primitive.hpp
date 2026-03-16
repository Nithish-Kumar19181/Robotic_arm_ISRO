#pragma once

#include "ur10e_force_control_base/force_controller.hpp"
#include "ur10e_force_control_base/kinematics_helper.hpp"
#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include <rclcpp/rclcpp.hpp>
using namespace kinematics_helper;

namespace motion_primitives {

class CirclePrimitive : public MotionPrimitiveBase {
public:
  CirclePrimitive() = default;

  void onStart(RobotState &robot_state, std::string &urdf_string) override;

  void update(RobotState &robot_state, double dt) override;

  void targetPose(geometry_msgs::msg::PoseStamped &pose) override;

  void targetWrench(geometry_msgs::msg::WrenchStamped &wrench) override;

  bool isDone() const override;

  double getProgress() const override;

  std::string getName() const override { return "CirclePrimitive"; }
  
  void setParameters(double angular_speed_, double max_angle_rad_,double target_force_, double direction_);

  void setForceGains(double kp, double ki, double integral_limit,double max_rate);

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
  double target_force_{-10.0};
  int direction_{-1}; // -1 for CW, 1 for CCW
  bool initialized_{false};
  bool isDone_{false};
  rclcpp::Time start_time_;
  double progress_{0.0};
  double measured_force_{0.0};
  double radius_dot_{0.0};

  double radius_min_{0.05};
  double radius_max_{1.5};

  force_control::ForceController force_ctrl_;
  KinematicsHelper kinematicsHelper_;
};

} // namespace motion_primitives
