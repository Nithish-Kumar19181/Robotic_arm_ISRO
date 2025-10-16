#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
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
      inward_distance_(0.05),    // 5 cm inward
      radial_step_(0.002),       // 2 mm per step
      line_step_(0.002),
      line_target_(0.1),         // 10 cm upward per cycle
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
    double radius_, z_height_, angular_vel_, t_;
    int loop_count_, max_loops_;
    double inward_distance_, radial_step_, line_step_, line_target_;
    double publish_time_s_;
    MotionState state_ = IDLE;

    double radial_progress_ = 0.0;
    double line_progress_ = 0.0;
    double current_radius_;

    KDL::Rotation last_orientation_ = KDL::Rotation::Identity();

    // ----------- Helper: create tool orientation ------------
    KDL::Rotation createRadialOrientation(double t, bool forward) {
        double x = current_radius_ * cos(t);
        double y = current_radius_ * sin(t);

        KDL::Vector radial(x, y, 0.0);
        if (radial.Norm() < 1e-8)
            radial = KDL::Vector(1.0, 0.0, 0.0);
        radial.Normalize();

        KDL::Vector z_axis = radial;
        KDL::Vector global_up(0.0, 0.0, 1.0);
        KDL::Vector y_axis = z_axis * global_up;
        y_axis.Normalize();

        if (!forward) y_axis = y_axis * (1.0);

        KDL::Vector x_axis = y_axis * z_axis;
        x_axis.Normalize();

        return KDL::Rotation(x_axis, y_axis, z_axis);
    }

    bool ik_ready() const { return (bool)ik_solver_; }

    // ----------- Callback: URDF received --------------------
    void on_urdf_received(const std_msgs::msg::String::SharedPtr msg) {
        if (ik_solver_) return; // already done

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
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from base_link to tool0.");
            return;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
        q_init_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
        for (unsigned int i = 0; i < q_init_.rows(); ++i) q_init_(i) = 0.0;

        // Initialize trajectory
        state_ = CIRCLE_FWD;
        t_ = 0.0;
        current_radius_ = radius_;
        radial_progress_ = 0.0;
        line_progress_ = 0.0;
        publish_time_s_ = 0.0;
        last_orientation_ = KDL::Rotation::Identity();

        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(0.01),
            std::bind(&CircleAndLineNode::publishNextPoint, this));

        RCLCPP_INFO(this->get_logger(),
                    "IK solver initialized. Starting forward circular motion.");
    }

    // ----------- Main trajectory update loop ----------------
    void publishNextPoint() {
        if (!ik_ready()) return;

        if (loop_count_ >= max_loops_) {
            RCLCPP_INFO(this->get_logger(),
                        "Completed %d loops. Stopping motion.", max_loops_);
            timer_->cancel();
            return;
        }

        publish_time_s_ += 0.01;

        KDL::Frame candidate_pose = KDL::Frame::Identity();

        // --- Compute pose for current motion phase (no state advance yet) ---
        if (state_ == CIRCLE_FWD) {
            double x = current_radius_ * cos(t_);
            double y = current_radius_ * sin(t_);
            double z = z_height_;
            candidate_pose = KDL::Frame(createRadialOrientation(t_, true), KDL::Vector(x, y, z));
        }
        else if (state_ == INWARD) {
            double progress_try = std::min(radial_progress_ + radial_step_, inward_distance_);
            double r = radius_ - progress_try;
            candidate_pose = KDL::Frame(last_orientation_, KDL::Vector(r, 0.0, z_height_));
        }
        else if (state_ == CIRCLE_REV) {
            double x = current_radius_ * cos(-t_);
            double y = current_radius_ * sin(-t_);
            double z = z_height_;
            candidate_pose = KDL::Frame(createRadialOrientation(-t_, false), KDL::Vector(x, y, z));
        }
        else if (state_ == OUTWARD) {
            double progress_try = std::min(radial_progress_ + radial_step_, inward_distance_);
            double r = (radius_ - inward_distance_) + progress_try;
            candidate_pose = KDL::Frame(last_orientation_, KDL::Vector(r, 0.0, z_height_));
        }
        else if (state_ == LINE_UP) {
            double progress_try = std::min(line_progress_ + line_step_, line_target_);
            candidate_pose = KDL::Frame(last_orientation_, KDL::Vector(radius_, 0.0, z_height_ + progress_try));
        }

        // --- Solve IK ---
        KDL::JntArray q_sol(kdl_chain_.getNrOfJoints());
        int ret = ik_solver_->CartToJnt(q_init_, candidate_pose, q_sol);

        if (ret >= 0) {
            // --- Success: publish and advance ---
            trajectory_msgs::msg::JointTrajectoryPoint pt;
            pt.positions.resize(q_sol.rows());
            for (unsigned int i = 0; i < q_sol.rows(); ++i)
                pt.positions[i] = q_sol(i);
            pt.time_from_start = rclcpp::Duration::from_seconds(publish_time_s_);
            pub_->publish(pt);
            q_init_ = q_sol;

            // --- Advance state after successful IK ---
            if (state_ == CIRCLE_FWD) {
                t_ += angular_vel_ * 0.01;
                last_orientation_ = candidate_pose.M;
                if (t_ >= 2*M_PI) {
                    t_ = 0.0;
                    state_ = INWARD;
                    radial_progress_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "CIRCLE_FWD complete → INWARD.");
                }
            }
            else if (state_ == INWARD) {
                radial_progress_ += radial_step_;
                if (radial_progress_ >= inward_distance_) {
                    radial_progress_ = inward_distance_;
                    current_radius_ = radius_ - inward_distance_;
                    state_ = CIRCLE_REV;
                    t_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "INWARD complete → CIRCLE_REV.");
                } else {
                    current_radius_ = radius_ - radial_progress_;
                }
            }
            else if (state_ == CIRCLE_REV) {
                t_ += angular_vel_ * 0.01;
                last_orientation_ = candidate_pose.M;
                if (t_ >= 2*M_PI) {
                    t_ = 0.0;
                    state_ = OUTWARD;
                    radial_progress_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "CIRCLE_REV complete → OUTWARD.");
                }
            }
            else if (state_ == OUTWARD) {
                radial_progress_ += radial_step_;
                if (radial_progress_ >= inward_distance_) {
                    current_radius_ = radius_;
                    state_ = LINE_UP;
                    line_progress_ = 0.0;
                    RCLCPP_INFO(this->get_logger(), "OUTWARD complete → LINE_UP.");
                } else {
                    current_radius_ = (radius_ - inward_distance_) + radial_progress_;
                }
            }
            else if (state_ == LINE_UP) {
                line_progress_ += line_step_;
                if (line_progress_ >= line_target_) {
                    z_height_ += line_target_;
                    loop_count_++;
                    current_radius_ = radius_;
                    t_ = 0.0;
                    state_ = CIRCLE_FWD;
                    RCLCPP_INFO(this->get_logger(),
                                "LINE_UP complete. Loop %d/%d done. Restarting circle.",
                                loop_count_, max_loops_);
                }
            }

        } else {
            // --- IK failed ---
            RCLCPP_WARN(this->get_logger(),
                        "IK failed (ret=%d) at state=%d pose=(%.3f, %.3f, %.3f).",
                        ret, (int)state_,
                        candidate_pose.p.x(), candidate_pose.p.y(), candidate_pose.p.z());
            return; // skip advancing
        }

        // --- RViz marker ---
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "base_link";
        marker.header.stamp = this->now();
        marker.ns = "ik_path";
        marker.id = loop_count_ * 100000 + static_cast<int>(publish_time_s_ * 1000) % 100000;
        marker.type = visualization_msgs::msg::Marker::ARROW;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = candidate_pose.p.x();
        marker.pose.position.y = candidate_pose.p.y();
        marker.pose.position.z = candidate_pose.p.z();

        double x, y, z, w;
        candidate_pose.M.GetQuaternion(x, y, z, w);
        marker.pose.orientation.x = x;
        marker.pose.orientation.y = y;
        marker.pose.orientation.z = z;
        marker.pose.orientation.w = w;

        marker.scale.x = 0.1;
        marker.scale.y = 0.02;
        marker.scale.z = 0.02;
        marker.color.a = 1.0;

        switch (state_) {
            case CIRCLE_FWD: marker.color.g = 1.0; break;
            case CIRCLE_REV: marker.color.r = 1.0; break;
            case INWARD:
            case OUTWARD: marker.color.b = 1.0; break;
            case LINE_UP: marker.color.r = marker.color.g = 1.0; break;
            default: break;
        }

        visualization_msgs::msg::MarkerArray arr;
        arr.markers.push_back(marker);
        marker_pub_->publish(arr);
    }
};

// ----------- Main ----------------
int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleAndLineNode>());
    rclcpp::shutdown();
    return 0;
}
