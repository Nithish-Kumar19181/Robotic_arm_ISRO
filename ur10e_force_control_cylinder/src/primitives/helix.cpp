#include "ur10e_force_control_base/force_controller.hpp"
#include "ur10e_force_control_base/kinematics_helper.hpp"
#include "ur10e_force_control_base/mapping/orientation_utils.hpp"
#include "ur10e_force_control_base/mapping/trajectory_math.hpp"
#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include "ur10e_force_control_cylinder/primitives/helix_primitive.hpp"
#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace trajectory_math;
using namespace orientation_utils;
using namespace force_control;
using namespace trajectory_math ;

namespace motion_primitives 
{

    void HelixPrimitive::onStart(RobotState &robot_state,std::string &urdf_string) 
    {
    if (initialized_) {
        return;
    }

    if (!kinematicsHelper_.isReady()) {
        if (!kinematicsHelper_.init(urdf_string, "base_link", "tool0"))
        return;
    }
    if (!kinematicsHelper_.computeFK(robot_state))
        return;

    kinematicsHelper_.getPosition(robot_state.frame_, start_x, start_y, start_z);
    kinematicsHelper_.getQuaternion(robot_state.frame_, qx_, qy_, qz_, qw_);

    radius_ = std::hypot(start_x, start_y);
    theta_ = std::atan2(start_y, start_x);
    start_time_ = rclcpp::Clock().now();

    elapsed_ = 0.0;
    traveled_angle_ = 0.0;
    progress_ = 0.0;
    isDone_ = false;

    initialized_ = true;
    }

    void HelixPrimitive::update(RobotState &robot_state, double dt)
    {
        if (!initialized_ || isDone_)
            return;

        if (dt <= 0.0)
            return;

        elapsed_ += dt;

        double t_base = max_angle_rad_ / angular_speed_;
        double ramp_time = accel_ratio_ * t_base;
        double total_time = t_base + ramp_time;

        double alpha = 1.0 ;

        double dtheta = direction_ * angular_speed_ * alpha * dt;
        theta_ += dtheta;

        traveled_angle_ += std::abs(dtheta);

        progress_ = std::clamp(traveled_angle_ / max_angle_rad_, 0.0, 1.0);

        if (traveled_angle_ >= max_angle_rad_)
            isDone_ = true;
    }

    void HelixPrimitive::targetPose(geometry_msgs::msg::PoseStamped &pose) {
    pose.header.frame_id = "base_link";
    pose.header.stamp = rclcpp::Clock().now();

    pose.pose.position.x = radius_ * std::cos(theta_);
    pose.pose.position.y = radius_ * std::sin(theta_);
    pose.pose.position.z = start_z + progress_ * target_height_;

    orientation_utils::computeRadialOrientation(theta_, qx_, qy_, qz_, qw_);

    pose.pose.orientation.x = qx_;
    pose.pose.orientation.y = qy_;
    pose.pose.orientation.z = qz_;
    pose.pose.orientation.w = qw_;
    }

    void HelixPrimitive::targetWrench(geometry_msgs::msg::WrenchStamped &wrench) {
    wrench.header.frame_id = "tool0";
    wrench.header.stamp = rclcpp::Clock().now();

    wrench.wrench.force.x = 0.0;
    wrench.wrench.force.y = 0.0;
    wrench.wrench.force.z = 3.0;

    wrench.wrench.torque.x = 0.0;
    wrench.wrench.torque.y = 0.0;
    wrench.wrench.torque.z = 0.0;

    if (isDone()) {
        wrench.wrench.force.z = 0.0;
    }
    }

    void HelixPrimitive::setParameters(double angular_speed, double max_angle_rad,
        double target_height, double direction) 
    {
    angular_speed_ = angular_speed;
    max_angle_rad_ = max_angle_rad;
    target_height_ = target_height;
    direction_ = direction ;
    }

    bool HelixPrimitive::isDone() const { return progress_ >= 1.0; }

    double HelixPrimitive::getProgress() const { return progress_; }

} // namespace motion_primitives
