#pragma once

#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include <rclcpp/rclcpp.hpp>
#include "ur10e_force_control_base/kinematics_helper.hpp"

using namespace kinematics_helper ;

namespace motion_primitives {

class ApproachPrimitive : public MotionPrimitiveBase {
public:
  ApproachPrimitive() = default;

  void onStart(RobotState &robot_state, std::string &urdf_string) override;

  void update(RobotState &robot_state, double dt) override;

  void targetPose(geometry_msgs::msg::PoseStamped &pose) override;

  void targetWrench(geometry_msgs::msg::WrenchStamped &wrench) override;

  bool isDone() const override;

  double getProgress() const override;

  std::string getName() const override { return "ApproachPrimitive"; }

  // Custom method to configure parameters
  void setParameters(double approach_distance, double approach_duration,
    double approach_force_z, double contact_threshold);

private:
  double start_x{0.0}, start_y{0.0}, start_z{0.0};
  double dir_x_{0.0}, dir_y_{0.0};
  double qx_{0.0}, qy_{0.0}, qz_{0.0}, qw_{1.0};

  rclcpp::Time start_time_;

  double distance_{0.35};
  double duration_{7.0};
  double force_z_{-2.0};
  double contact_threshold_{25.0};

  bool contact_detected_{false};
  bool initialized_{false};
  double progress_{0.0};
  KinematicsHelper kinematicsHelper_ ;
};

} // namespace motion_primitives
