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

        start_x_ = tcp.p.x()+0.001;
        start_y_ = tcp.p.y()+0.001;
        start_z_ = tcp.p.z();

        radius_ = std::hypot(start_x_, start_y_);
        start_theta_ = std::atan2(start_y_, start_x_);

        RCLCPP_INFO(get_logger(),
            "Start TCP: (%.3f, %.3f, %.3f), r=%.3f",
            start_x_, start_y_, start_z_, radius_);

        auto t = this->create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() { generateTrajectory(); });

        timers_.push_back(t);
    }

    void generateTrajectory()
    {
        if (executed_) return;

        trajectory_msgs::msg::JointTrajectory traj;
        traj.joint_names = ur_joints_;
        traj.header.stamp = this->now() + rclcpp::Duration::from_seconds(0.5);

        const int N = 360;

        KDL::JntArray q_seed = q_current_;
        double t_total = 0.0;

        for (int i = 0; i <= N; i++)
        {
            double theta = start_theta_ + (2.0 * M_PI * i / N);

            double xd = radius_ * std::cos(theta);
            double yd = radius_ * std::sin(theta);
            double zd = start_z_;

            // Orientation: radial
            KDL::Vector z_axis(std::cos(theta), std::sin(theta), 0.0);
            z_axis.Normalize();
            KDL::Vector up(0,0,1);
            KDL::Vector x_axis = up * z_axis;
            x_axis.Normalize();
            KDL::Vector y_axis = z_axis * x_axis;
            y_axis.Normalize();

            KDL::Rotation R(
                x_axis.x(), y_axis.x(), z_axis.x(),
                x_axis.y(), y_axis.y(), z_axis.y(),
                x_axis.z(), y_axis.z(), z_axis.z()
            );

            KDL::Frame target(R, KDL::Vector(xd, yd, zd));
            KDL::JntArray q_next(6);

            if (ik_->CartToJnt(q_seed, target, q_next) < 0)
                continue;

            double dq_max = 0.0;
            for (int j = 0; j < 6; j++)
                dq_max = std::max(dq_max, std::fabs(q_next(j) - q_seed(j)));

            double dt = std::max(dq_max / v_max_, 0.05);  // minimum 50 ms
            t_total += dt;
            // -------------------------------------------------------

            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(6);
            for (int j = 0; j < 6; j++) pt.positions[j] = q_next(j);

            // DO NOT set velocities / accelerations

            pt.time_from_start.sec = (int)t_total;
            pt.time_from_start.nanosec =
                (int)((t_total - (int)t_total) * 1e9);

            traj.points.push_back(pt);
            q_seed = q_next;
        }

        traj_pub_->publish(traj);

        RCLCPP_INFO(get_logger(),
            "Published safe trajectory with %zu points, duration %.2f s",
            traj.points.size(), t_total);

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
