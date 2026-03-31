#include "ur10e_force_control_base/force_controller.hpp"
#include "ur10e_force_control_base/kinematics_helper.hpp"
#include "ur10e_force_control_base/mapping/trajectory_math.hpp"
#include "ur10e_force_control_cylinder/primitives/horizontal_primitive.hpp"

#include <algorithm>
#include <cmath>
#include <rclcpp/rclcpp.hpp>

using namespace trajectory_math;
using namespace force_control;

namespace motion_primitives {

    void HorizontalPrimitive::onStart(RobotState &robot_state,std::string &urdf_string)
    {
        if (initialized_)
            return;

        if (!kinematicsHelper_.isReady())
        {
            if (!kinematicsHelper_.init(urdf_string, "base_link", "tool0"))
                return;
        }

        if (!kinematicsHelper_.computeFK(robot_state))
            return;

        // Get initial pose
        double start_x, start_y, start_z;
        kinematicsHelper_.getPosition(robot_state.frame_, start_x, start_y, start_z);
        kinematicsHelper_.getQuaternion(robot_state.frame_, qx_, qy_, qz_, qw_);

        current_x_ = start_x;
        current_y_ = start_y;
        start_z_ = start_z;

        traveled_distance_ = 0.0;
        elapsed_ = 0.0;
        progress_ = 0.0;
        isDone_ = false;

        force_ctrl_.resetForceControl();

        start_time_ = rclcpp::Clock().now();
        initialized_ = true;
    }

    void HorizontalPrimitive::update(RobotState &robot_state, double dt)
    {
        if (!initialized_ || isDone_ || dt <= 0.0)
            return;

        measured_force_ = robot_state.wrench_z_.wrench.force.z;

        elapsed_ += dt;

        double total_time = max_distance_ / linear_speed_;
        double ramp_time = accel_ratio_ * total_time;

        double alpha = 1.0;

        // if (elapsed_ < ramp_time)
        // {
        //     alpha = Scurve(elapsed_ / ramp_time);
        // }
        // else if (elapsed_ > total_time - ramp_time)
        // {
        //     double ratio = (total_time - elapsed_) / ramp_time;
        //     alpha = Scurve(std::clamp(ratio, 0.0, 1.0));
        // }

        force_ctrl_.computeAdmittance(measured_force_, target_force_, dt, x_dot_);
        current_x_ -= x_dot_ * dt;
        current_x_ = std::clamp(current_x_, x_min_, x_max_);

        double dy = direction_ * linear_speed_ * alpha * dt;
        current_y_ += dy;
        traveled_distance_ += std::abs(dy);

        progress_ = std::clamp(traveled_distance_ / max_distance_, 0.0, 1.0);

        if (traveled_distance_ >= max_distance_)
            isDone_ = true;
    }

    void HorizontalPrimitive::targetPose(geometry_msgs::msg::PoseStamped &pose)
    {
        pose.header.frame_id = "base_link";
        pose.header.stamp = rclcpp::Clock().now();

        pose.pose.position.x = current_x_;
        pose.pose.position.y = current_y_;
        pose.pose.position.z = start_z_;

        pose.pose.orientation.x = qx_;
        pose.pose.orientation.y = qy_;
        pose.pose.orientation.z = qz_;
        pose.pose.orientation.w = qw_;
    }

    void HorizontalPrimitive::targetWrench(geometry_msgs::msg::WrenchStamped &wrench)
    {
        wrench.header.frame_id = "tool0";
        wrench.header.stamp = rclcpp::Clock().now();

        wrench.wrench.force.x = 0.0;
        wrench.wrench.force.y = 0.0;
        wrench.wrench.force.z = 3.0;

        wrench.wrench.torque.x = 0.0;
        wrench.wrench.torque.y = 0.0;
        wrench.wrench.torque.z = 0.0;

        if (isDone())
            wrench.wrench.force.z = 0.0;
    }

    void HorizontalPrimitive::setParameters(double linear_speed,double max_distance,double target_force,int direction)
    {
        linear_speed_ = linear_speed;
        max_distance_ = max_distance;
        target_force_ = target_force;
        direction_ = direction;
    }

    void HorizontalPrimitive::setForceGains(double kp, double ki,double integral_limit, double max_rate)
    {
        force_ctrl_ = force_control::ForceController(kp, ki, integral_limit, max_rate);
    }

    bool HorizontalPrimitive::isDone() const
    {
        return progress_ >= 1.0;
    }

    double HorizontalPrimitive::getProgress() const
    {
        return progress_;
    }

} // namespace motion_primitives