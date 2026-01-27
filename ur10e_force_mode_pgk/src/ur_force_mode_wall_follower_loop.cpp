#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

#include <cmath>
#include <map>
#include <vector>
#include <algorithm>

class CircularMotionNode : public rclcpp::Node
{
public:
    CircularMotionNode() : Node("circular_motion_node")
    {
        RCLCPP_INFO(get_logger(), "Circular Motion Node Started");

        ur_joints_ = {
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&CircularMotionNode::onURDF, this, std::placeholders::_1));

        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CircularMotionNode::onJointState, this, std::placeholders::_1));

        traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            "/joint_trajectory_controller/joint_trajectory", 10);

        v_max_ = 0.1;   // slow and safe
    }

private:

    void onURDF(const std_msgs::msg::String::SharedPtr msg)
    {
        if (urdf_loaded_) return;

        urdf::Model model;
        model.initString(msg->data);

        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(model, tree)) {
            RCLCPP_ERROR(get_logger(), "URDF → KDL tree failed");
            return;
        }

        if (!tree.getChain("base_link", "tool0", chain_)) {
            RCLCPP_ERROR(get_logger(), "Failed to get KDL chain base_link→tool0");
            return;
        }

        fk_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        ik_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);

        q_current_ = KDL::JntArray(6);
        urdf_loaded_ = true;

        RCLCPP_INFO(get_logger(), "URDF Loaded → KDL Ready");
    }

    void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!urdf_loaded_ || executed_) return;

        std::map<std::string, double> m;
        for (size_t i = 0; i < msg->name.size(); i++)
            m[msg->name[i]] = msg->position[i];

        for (size_t i = 0; i < 6; i++) {
            if (!m.count(ur_joints_[i])) return;
            q_current_(i) = m[ur_joints_[i]];
        }

        KDL::Frame tcp;
        fk_->JntToCart(q_current_, tcp);

        start_x_ = tcp.p.x() + 0.001;
        start_y_ = tcp.p.y() + 0.001;
        start_z_ = tcp.p.z();
        
        circle_center_x_ = 0.0;
        circle_center_y_ = 0.0;
        radius_ = std::hypot(start_x_, start_y_);
        start_theta_ = std::atan2(start_y_, start_x_);

        RCLCPP_INFO(get_logger(),
            "Start TCP: (%.3f, %.3f, %.3f), r=%.3f, theta=%.3f rad",
            start_x_, start_y_, start_z_, radius_, start_theta_);

        auto t = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { generateTrajectorySequence(); });

        timers_.push_back(t);
    }

    void generateTrajectorySequence()
    {
        if (executed_) return;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = ur_joints_;
        traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.5);

        KDL::JntArray q_seed = q_current_;
        double t_total = 0.0;
        
        KDL::Frame current_frame;
        fk_->JntToCart(q_current_, current_frame);
        
        RCLCPP_INFO(get_logger(), "Generating CW rotation...");
        int cw_steps = 72;  
        std::vector<KDL::JntArray> cw_joint_states;
        
        for (int i = 0; i <= cw_steps; i++)
        {
            double fraction = (double)i / cw_steps;
            double theta = start_theta_ + (2.0 * M_PI * fraction);  // CW
            
            double xd = radius_ * std::cos(theta);
            double yd = radius_ * std::sin(theta);
            double zd = start_z_;
          
            KDL::Vector radial(std::cos(theta), std::sin(theta), 0.0);
            radial.Normalize();
            
            // Tool Y-axis points upward (Z)
            KDL::Vector up(0, 0, 1);
            
            KDL::Vector tangential = up * radial;
            tangential.Normalize();
            
            KDL::Rotation R(
                tangential.x(), up.x(), radial.x(),
                tangential.y(), up.y(), radial.y(),
                tangential.z(), up.z(), radial.z()
            );
            
            KDL::Frame target(R, KDL::Vector(xd, yd, zd));
            KDL::JntArray q_next(6);

            if (ik_->CartToJnt(q_seed, target, q_next) < 0)
            {
                RCLCPP_WARN(get_logger(), "IK failed at CW step %d, theta=%.3f", i, theta);
                continue;
            }

            double dq_max = 0.0;
            for (int j = 0; j < 6; j++)
                dq_max = std::max(dq_max, std::fabs(q_next(j) - q_seed(j)));

            double dt = std::max(dq_max / v_max_, 0.05);
            t_total += dt;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(6);
            for (int j = 0; j < 6; j++) pt.positions[j] = q_next(j);

            pt.time_from_start.sec = (int)t_total;
            pt.time_from_start.nanosec = (int)((t_total - (int)t_total) * 1e9);

            traj.points.push_back(pt);
            q_seed = q_next;
            cw_joint_states.push_back(q_next);
        }
        
        // Add 2-second pause at end of CW rotation
        t_total += 2.0;
        if (!cw_joint_states.empty()) {
            trajectory_msgs::msg::JointTrajectoryPoint pause_pt;
            pause_pt.positions.resize(6);
            for (int j = 0; j < 6; j++) pause_pt.positions[j] = q_seed(j);
            pause_pt.time_from_start.sec = (int)t_total;
            pause_pt.time_from_start.nanosec = (int)((t_total - (int)t_total) * 1e9);
            traj.points.push_back(pause_pt);
        }
        
        RCLCPP_INFO(get_logger(), "Generating +X movement...");
        double x_offset = 0.1;
        int x_steps = 10;
        
        double final_theta_cw = start_theta_ + 2.0 * M_PI;  // Full circle
        KDL::Vector final_radial(std::cos(final_theta_cw), std::sin(final_theta_cw), 0.0);
        final_radial.Normalize();
        KDL::Vector up(0, 0, 1);
        KDL::Vector final_tangential = up * final_radial;
        final_tangential.Normalize();
        
        for (int i = 1; i <= x_steps; i++)
        {
            double fraction = (double)i / x_steps;
            double xd = start_x_ + x_offset * fraction;
            double yd = start_y_;
            double zd = start_z_;
            
            KDL::Rotation R(
                final_tangential.x(), up.x(), final_radial.x(),
                final_tangential.y(), up.y(), final_radial.y(),
                final_tangential.z(), up.z(), final_radial.z()
            );
            
            KDL::Frame target(R, KDL::Vector(xd, yd, zd));
            KDL::JntArray q_next(6);

            if (ik_->CartToJnt(q_seed, target, q_next) < 0)
            {
                RCLCPP_WARN(get_logger(), "IK failed at +X step %d", i);
                continue;
            }

            double dq_max = 0.0;
            for (int j = 0; j < 6; j++)
                dq_max = std::max(dq_max, std::fabs(q_next(j) - q_seed(j)));

            double dt = std::max(dq_max / v_max_, 0.05);
            t_total += dt;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(6);
            for (int j = 0; j < 6; j++) pt.positions[j] = q_next(j);

            pt.time_from_start.sec = (int)t_total;
            pt.time_from_start.nanosec = (int)((t_total - (int)t_total) * 1e9);

            traj.points.push_back(pt);
            q_seed = q_next;
        }

        t_total += 2.0;
        trajectory_msgs::msg::JointTrajectoryPoint pause_pt2;
        pause_pt2.positions.resize(6);
        for (int j = 0; j < 6; j++) pause_pt2.positions[j] = q_seed(j);
        pause_pt2.time_from_start.sec = (int)t_total;
        pause_pt2.time_from_start.nanosec = (int)((t_total - (int)t_total) * 1e9);
        traj.points.push_back(pause_pt2);
        
        RCLCPP_INFO(get_logger(), "Generating CCW rotation...");
        double new_center_x = start_x_ + x_offset;
        double new_center_y = start_y_;
        double new_radius = std::hypot(new_center_x, new_center_y);
        double center_theta = std::atan2(new_center_y, new_center_x);
        
        int ccw_steps = 72;  // 5-degree steps
        
        for (int i = 0; i <= ccw_steps; i++)
        {
            double fraction = (double)i / ccw_steps;
            double theta = center_theta - (2.0 * M_PI * fraction);  // CCW
            
            // Position on new circle
            double xd = new_radius * std::cos(theta);
            double yd = new_radius * std::sin(theta);
            double zd = start_z_;
            
            double radial_theta = std::atan2(yd, xd);  // Angle from origin
            KDL::Vector radial(std::cos(radial_theta), std::sin(radial_theta), 0.0);
            radial.Normalize();
            
            // Same orientation frame as before
            KDL::Vector tangential = up * radial;
            tangential.Normalize();
            
            KDL::Rotation R(
                tangential.x(), up.x(), radial.x(),
                tangential.y(), up.y(), radial.y(),
                tangential.z(), up.z(), radial.z()
            );
            
            KDL::Frame target(R, KDL::Vector(xd, yd, zd));
            KDL::JntArray q_next(6);

            if (ik_->CartToJnt(q_seed, target, q_next) < 0)
            {
                RCLCPP_WARN(get_logger(), "IK failed at CCW step %d, theta=%.3f", i, theta);
                continue;
            }

            double dq_max = 0.0;
            for (int j = 0; j < 6; j++)
                dq_max = std::max(dq_max, std::fabs(q_next(j) - q_seed(j)));

            double dt = std::max(dq_max / v_max_, 0.05);
            t_total += dt;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(6);
            for (int j = 0; j < 6; j++) pt.positions[j] = q_next(j);

            pt.time_from_start.sec = (int)t_total;
            pt.time_from_start.nanosec = (int)((t_total - (int)t_total) * 1e9);

            traj.points.push_back(pt);
            q_seed = q_next;
        }
        
        t_total += 2.0;
        trajectory_msgs::msg::JointTrajectoryPoint pause_pt3;
        pause_pt3.positions.resize(6);
        for (int j = 0; j < 6; j++) pause_pt3.positions[j] = q_seed(j);
        pause_pt3.time_from_start.sec = (int)t_total;
        pause_pt3.time_from_start.nanosec = (int)((t_total - (int)t_total) * 1e9);
        traj.points.push_back(pause_pt3);
        
        RCLCPP_INFO(get_logger(), "Generating +Z movement...");
        double z_offset = 0.05;
        int z_steps = 10;
        
        double final_theta_ccw = center_theta - 2.0 * M_PI;
        double final_x_ccw = new_radius * std::cos(final_theta_ccw);
        double final_y_ccw = new_radius * std::sin(final_theta_ccw);
        double final_radial_theta = std::atan2(final_y_ccw, final_x_ccw);
        
        KDL::Vector final_radial_ccw(std::cos(final_radial_theta), std::sin(final_radial_theta), 0.0);
        final_radial_ccw.Normalize();
        KDL::Vector final_tangential_ccw = up * final_radial_ccw;
        final_tangential_ccw.Normalize();
        
        for (int i = 1; i <= z_steps; i++)
        {
            double fraction = (double)i / z_steps;
            double xd = final_x_ccw;
            double yd = final_y_ccw;
            double zd = start_z_ + z_offset * fraction;
            
            KDL::Rotation R(
                final_tangential_ccw.x(), up.x(), final_radial_ccw.x(),
                final_tangential_ccw.y(), up.y(), final_radial_ccw.y(),
                final_tangential_ccw.z(), up.z(), final_radial_ccw.z()
            );
            
            KDL::Frame target(R, KDL::Vector(xd, yd, zd));
            KDL::JntArray q_next(6);

            if (ik_->CartToJnt(q_seed, target, q_next) < 0)
            {
                RCLCPP_WARN(get_logger(), "IK failed at +Z step %d", i);
                continue;
            }

            double dq_max = 0.0;
            for (int j = 0; j < 6; j++)
                dq_max = std::max(dq_max, std::fabs(q_next(j) - q_seed(j)));

            double dt = std::max(dq_max / v_max_, 0.05);
            t_total += dt;

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(6);
            for (int j = 0; j < 6; j++) pt.positions[j] = q_next(j);

            pt.time_from_start.sec = (int)t_total;
            pt.time_from_start.nanosec = (int)((t_total - (int)t_total) * 1e9);

            traj.points.push_back(pt);
            q_seed = q_next;
        }

        traj_pub_->publish(traj);
        executed_ = true;
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    std::vector<rclcpp::TimerBase::SharedPtr> timers_;

    KDL::Chain chain_;
    std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_;
    KDL::JntArray q_current_;

    bool urdf_loaded_ = false;
    bool executed_ = false;

    double start_x_, start_y_, start_z_;
    double circle_center_x_, circle_center_y_;
    double start_theta_;
    double radius_;

    double v_max_;
    std::vector<std::string> ur_joints_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircularMotionNode>());
    rclcpp::shutdown();
    return 0;
}