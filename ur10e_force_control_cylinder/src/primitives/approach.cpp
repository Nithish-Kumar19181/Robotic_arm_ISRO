#include "ur10e_force_control_base/kinematics_helper.hpp"
#include "ur10e_force_control_base/mapping/trajectory_math.hpp"
#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include "ur10e_force_control_cylinder/primitives/approach_primitive.hpp"

using namespace trajectory_math;

namespace motion_primitives {

    void ApproachPrimitive::onStart(RobotState &robot_state,std::string &urdf_string) 
    {
        if (initialized_) 
        {
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

        KDL::Vector z_axis = robot_state.frame_.M.UnitZ();

        dir_x_ = z_axis.x();
        dir_y_ = z_axis.y();

        start_time_ = rclcpp::Clock().now();

        initialized_ = true;
    }

    void ApproachPrimitive::update(RobotState &robot_state, double now) 
    {
        progress_ = (now - start_time_.seconds()) / duration_;
        progress_ = std::clamp(progress_, 0.0, 1.0);

        if (std::abs(robot_state.wrench_z_.wrench.force.z) > contact_threshold_) 
        {
            contact_detected_ = true;
        }
    }

    void ApproachPrimitive::targetPose(geometry_msgs::msg::PoseStamped &pose)
    {
        pose.header.frame_id = "base_link";
        pose.header.stamp = rclcpp::Clock().now();

        pose.pose.position.x = start_x + progress_ * distance_ * dir_x_;
        pose.pose.position.y = start_y + progress_ * distance_ * dir_y_;
        pose.pose.position.z = start_z;

        pose.pose.orientation.x = qx_;
        pose.pose.orientation.y = qy_;
        pose.pose.orientation.z = qz_;
        pose.pose.orientation.w = qw_;
    }

    void ApproachPrimitive::targetWrench(geometry_msgs::msg::WrenchStamped &wrench)
    {
        wrench.header.frame_id = "tool0";
        wrench.header.stamp = rclcpp::Clock().now();

        wrench.wrench.force.x = 0.0;
        wrench.wrench.force.y = 0.0;
        wrench.wrench.force.z = 0.0;

        wrench.wrench.torque.x = 0.0;
        wrench.wrench.torque.y = 0.0;
        wrench.wrench.torque.z = 0.0;
    }

    void ApproachPrimitive::setParameters(double approach_distance,double approach_duration,double approach_force_z,
        double contact_threshold) 
    {
        distance_ = approach_distance;
        duration_ = approach_duration;
        force_z_ = approach_force_z;
        contact_threshold_ = contact_threshold;
    }

    bool ApproachPrimitive::isDone() const 
    {
    return progress_ >= 1.0 || contact_detected_;
    }

double ApproachPrimitive::getProgress() const { return progress_; }
} // namespace motion_primitives
