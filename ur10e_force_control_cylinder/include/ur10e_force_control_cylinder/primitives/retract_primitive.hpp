#pragma once

#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include <rclcpp/rclcpp.hpp>

namespace motion_primitives {

class ApproachPrimitive : public MotionPrimitiveBase {
public:
  ApproachPrimitive() = default;

  void initialize(const KDL::Frame &start_frame, rclcpp::Time now) override;
  geometry_msgs::msg::PoseStamped computePose(rclcpp::Time now) override;
  geometry_msgs::msg::WrenchStamped computeWrench(rclcpp::Time now) override;
  bool isDone() const override;
  double getProgress() const override;
  std::string getName() const override { return "RetractPrimitive"; }

  // Custom method to configure parameters
  void setParameters(double retract_distance, double retract_duration);

  // Call this from the action server when new force measurements arrive
  void updateMeasuredForce(double fz);

private:
  double start_x_{0.0}, start_y_{0.0}, start_z_{0.0};
  double dir_x_{0.0}, dir_y_{0.0};
  double qx_{0.0}, qy_{0.0}, qz_{0.0}, qw_{1.0};

  rclcpp::Time start_time_;

  // Parameters
  double distance_{0.35};
  double duration_{7.0};
  double contact_threshold_{2.0};

  // State
  double current_progress_{0.0};
};

} // namespace motion_primitives
