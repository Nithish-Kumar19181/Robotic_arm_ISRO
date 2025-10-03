#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <cmath>
#include <memory>
#include <string>

using std::placeholders::_1;

class CircleAndLineNode : public rclcpp::Node {
public:
    CircleAndLineNode()
    : Node("circle_and_line_node"),
      radius_(0.5),
      z_height_(0.5),
      angular_vel_(0.2),         // rad/s
      t_(0.0),
      loop_count_(0),
      max_loops_(5),
      inward_distance_(0.2),    // 20 cm toward center
      radial_step_(0.002),      // 2 mm per step
      line_step_(0.002),        // 2 mm per step for vertical move
      line_target_(0.1),        // 10 cm up
      publish_time_s_(0.0)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for /robot_description...");

        urdf_sub_ = this->create_subscription<std_msgs::msg::String>(
            "robot_description",
            rclcpp::QoS(rclcpp::KeepLast(1))
                .durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
            std::bind(&CircleAndLineNode::on_urdf_received, this, _1));

        pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/target_joint_positions", 10);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/ik_trajectory_markers", 10);
    }

private:
    enum MotionState { IDLE=0, CIRCLE_FWD, INWARD, CIRCLE_REV, OUTWARD, LINE_UP };

    // ROS entities
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // KDL
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::JntArray q_init_;

    // Parameters / internal state
    double radius_;
    double z_height_;
    double angular_vel_;
    double t_;
    int loop_count_;
    int max_loops_;

    double inward_distance_;
    double radial_step_;
    double line_step_;
    double line_target_;

    double publish_time_s_; // monotonic time for time_from_start in messages

    MotionState state_ = IDLE;
    double radial_progress_ = 0.0; // used for INWARD/OUTWARD
    double line_progress_ = 0.0;   // used for LINE_UP
    double current_radius_;

    // Helpers
    bool ik_ready() const { return (bool)ik_solver_; }

    void on_urdf_received(const std_msgs::msg::String::SharedPtr msg) {
        if (ik_solver_) return; // already initialized

        urdf::Model model;
        if (!model.initString(msg->data)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to parse URDF.");
            return;
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromUrdfModel(model, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL tree.");
            return;
        }

        if (!kdl_tree.getChain("base_link", "tool0", kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from base_link to tool0");
            return;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
        q_init_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
        for (unsigned int i = 0; i < q_init_.rows(); ++i) q_init_(i) = 0.0;

        // Start in forward circle state
        state_ = CIRCLE_FWD;
        t_ = 0.0;
        current_radius_ = radius_;
        radial_progress_ = 0.0;
        line_progress_ = 0.0;
        publish_time_s_ = 0.0;

        // timer at 100 Hz
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.01),
            std::bind(&CircleAndLineNode::publishNextPoint, this));

        RCLCPP_INFO(this->get_logger(), "IK solver initialized. Starting forward circle motion.");
    }

    void publishNextPoint() {
        if (!ik_ready()) return;

        if (loop_count_ >= max_loops_) {
            RCLCPP_INFO(this->get_logger(), "Completed %d loops. Stopping.", max_loops_);
            timer_->cancel();
            return;
        }

        // Increment publish time
        publish_time_s_ += 0.01;

        KDL::Frame pose = KDL::Frame::Identity();

        if (state_ == CIRCLE_FWD) {
            double x = current_radius_ * cos(t_);
            double y = current_radius_ * sin(t_);
            double z = z_height_;

            KDL::Vector radial(x, y, 0.0);
            if (radial.Norm() < 1e-8) radial = KDL::Vector(1.0, 0.0, 0.0);
            radial.Normalize();
            KDL::Vector z_axis(0, 0, 1);
            KDL::Vector y_axis = z_axis * radial;
            KDL::Rotation rot(radial, y_axis, z_axis);

            pose = KDL::Frame(rot, KDL::Vector(x, y, z));

            t_ += angular_vel_ * 0.01;
            if (t_ >= 2*M_PI) {
                t_ = 0.0;
                state_ = INWARD;
                radial_progress_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "CIRCLE_FWD complete. Switching to INWARD.");
            }
        }
        else if (state_ == INWARD) {
            radial_progress_ += radial_step_;
            if (radial_progress_ > inward_distance_) radial_progress_ = inward_distance_;
            current_radius_ = radius_ - radial_progress_;
            double x = current_radius_;
            double y = 0.0;
            double z = z_height_;
            pose = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(x, y, z));

            if (radial_progress_ >= inward_distance_) {
                state_ = CIRCLE_REV;
                t_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "INWARD complete. Starting CIRCLE_REV.");
            }
        }
        else if (state_ == CIRCLE_REV) {
            double x = current_radius_ * cos(-t_);
            double y = current_radius_ * sin(-t_);
            double z = z_height_;

            KDL::Vector radial(x, y, 0.0);
            radial.Normalize();
            KDL::Vector z_axis(0, 0, 1);
            KDL::Vector y_axis = z_axis * radial;
            KDL::Rotation rot(radial, y_axis, z_axis);

            pose = KDL::Frame(rot, KDL::Vector(x, y, z));

            t_ += angular_vel_ * 0.01;
            if (t_ >= 2*M_PI) {
                t_ = 0.0;
                state_ = OUTWARD;
                radial_progress_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "CIRCLE_REV complete. Switching to OUTWARD.");
            }
        }
        else if (state_ == OUTWARD) {
            radial_progress_ += radial_step_;
            if (radial_progress_ > inward_distance_) radial_progress_ = inward_distance_;
            current_radius_ = (radius_ - inward_distance_) + radial_progress_;
            double x = current_radius_;
            double y = 0.0;
            double z = z_height_;
            pose = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(x, y, z));

            if (radial_progress_ >= inward_distance_) {
                state_ = LINE_UP;
                line_progress_ = 0.0;
                RCLCPP_INFO(this->get_logger(), "OUTWARD complete. Switching to LINE_UP.");
            }
        }
        else if (state_ == LINE_UP) {
            line_progress_ += line_step_;
            if (line_progress_ > line_target_) line_progress_ = line_target_;
            double x = radius_;
            double y = 0.0;
            double z = z_height_ + line_progress_;
            pose = KDL::Frame(KDL::Rotation::Identity(), KDL::Vector(x, y, z));

            if (line_progress_ >= line_target_) {
                z_height_ += line_target_;
                state_ = CIRCLE_FWD;
                t_ = 0.0;
                loop_count_++;
                current_radius_ = radius_;
                RCLCPP_INFO(this->get_logger(), "LINE_UP complete. Loop %d/%d. Restarting CIRCLE_FWD.", loop_count_, max_loops_);
            }
        }

        // Solve IK
        KDL::JntArray q_sol(kdl_chain_.getNrOfJoints());
        int ret = ik_solver_->CartToJnt(q_init_, pose, q_sol);
        if (ret >= 0) {
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(q_sol.rows());
            for (unsigned int i = 0; i < q_sol.rows(); ++i)
                pt.positions[i] = q_sol(i);
            pt.time_from_start = rclcpp::Duration::from_seconds(publish_time_s_);
            pub_->publish(pt);
            q_init_ = q_sol;
        }

        // Markers
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "ik_path";
        marker.id = loop_count_*100000 + (int)(publish_time_s_*1000)%100000;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = pose.p.x();
        marker.pose.position.y = pose.p.y();
        marker.pose.position.z = pose.p.z();
        marker.scale.x = marker.scale.y = marker.scale.z = 0.02;
        marker.color.a = 1.0;

        if (state_ == CIRCLE_FWD) marker.color.g = 1.0;
        else if (state_ == CIRCLE_REV) marker.color.r = 1.0;
        else if (state_ == INWARD || state_ == OUTWARD) marker.color.b = 1.0;
        else if (state_ == LINE_UP) { marker.color.r = 1.0; marker.color.g = 1.0; }
        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(marker);
        marker_pub_->publish(arr);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleAndLineNode>());
    rclcpp::shutdown();
    return 0;
}
