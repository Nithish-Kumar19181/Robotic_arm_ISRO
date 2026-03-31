#pragma once

#include "ur10e_force_control_base/force_controller.hpp"
#include "ur10e_force_control_base/kinematics_helper.hpp"
#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include <rclcpp/rclcpp.hpp>

using namespace kinematics_helper;

namespace motion_primitives {

class HorizontalPrimitive : public MotionPrimitiveBase {
public:
  HorizontalPrimitive() = default;

  void onStart(RobotState &robot_state, std::string &urdf_string) override;

  void update(RobotState &robot_state, double dt) override;

  void targetPose(geometry_msgs::msg::PoseStamped &pose) override;

  void targetWrench(geometry_msgs::msg::WrenchStamped &wrench) override;

  bool isDone() const override;

  double getProgress() const override;

  std::string getName() const override { return "HorizontalPrimitive"; }

  void setParameters(double linear_speed,
                     double max_distance,
                     double target_force,
                     int direction);

  void setForceGains(double kp, double ki,
                     double integral_limit, double max_rate);

private:
  double current_x_{0.0};
  double current_y_{0.0};
  double start_z_{0.0};

  double qx_{0.0}, qy_{0.0}, qz_{0.0}, qw_{1.0};

  double traveled_distance_{0.0};
  double linear_speed_{0.05};
  double max_distance_{0.5};
  double elapsed_{0.0};
  double accel_ratio_{0.15};

  double target_force_{-10.0};
  double measured_force_{0.0};
  double x_dot_{0.0};

  double x_min_{-1.5};
  double x_max_{1.5};

  int direction_{1}; // +1 (along +Y), -1 (along -Y)

  bool initialized_{false};
  bool isDone_{false};
  double progress_{0.0};

  rclcpp::Time start_time_;

  force_control::ForceController force_ctrl_;
  KinematicsHelper kinematicsHelper_;
};

} // namespace motion_primitives