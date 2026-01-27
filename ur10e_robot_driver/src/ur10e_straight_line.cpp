#include "rclcpp/rclcpp.hpp"
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

using namespace std::placeholders;

class CartesianMotionController : public rclcpp::Node
{
public:
  CartesianMotionController() : Node("cartesian_motion_controller")
  { 
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (urdf_loaded_) return;
        
        RCLCPP_INFO(get_logger(), "Received URDF");
        urdf::Model model;
        if (!model.initString(msg->data)) {
          RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
          return;
        }

        KDL::Tree tree;
        if (!kdl_parser::treeFromUrdfModel(model, tree)) {
          RCLCPP_ERROR(get_logger(), "Failed to create KDL tree");
          return;
        }

        if (!tree.getChain("base_link", "tool0", chain_)) {
          RCLCPP_ERROR(get_logger(), "Failed to get chain");
          return;
        }

        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
        fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
        q_current_ = KDL::JntArray(6);
        urdf_loaded_ = true;
        RCLCPP_INFO(get_logger(), "URDF loaded successfully");
      });
      
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        if (!urdf_loaded_ || motion_started_) return;
        
        if (msg->position.size() < 6) {
          RCLCPP_ERROR(get_logger(), "Expected 6 joints, got %zu", msg->position.size());
          return;
        }
      
        q_current_(0) = msg->position[5];  // shoulder_pan_joint
        q_current_(1) = msg->position[0];  // shoulder_lift_joint
        q_current_(2) = msg->position[1];  // elbow_joint
        q_current_(3) = msg->position[2];  // wrist_1_joint
        q_current_(4) = msg->position[3];  // wrist_2_joint
        q_current_(5) = msg->position[4];  // wrist_3_joint
        
        RCLCPP_INFO(get_logger(), "Mapped joints to KDL order");
        
        // Compute current pose
        KDL::Frame current_pose;
        if (fk_solver_->JntToCart(q_current_, current_pose) < 0) {
          RCLCPP_ERROR(get_logger(), "Failed to compute FK");
          return;
        }
        
        start_pos_ = current_pose.p;
        start_orientation_ = current_pose.M;
    
        target_pos_ = start_pos_ + KDL::Vector(-0.20, 0.0, 0.0);
        total_distance_ = (target_pos_ - start_pos_).Norm();
        
        if (total_distance_ < 0.0001) {
          return;
        }
        
        motion_started_ = true;
        
        motion_timer_ = create_wall_timer(
          std::chrono::milliseconds(50),
          [this]() {
            if (progress_ >= 1.0) {
              if (motion_timer_) {
                motion_timer_->cancel();
              }
              return;
            }
            
            double clamped = std::min(progress_, 1.0);
            KDL::Vector target = start_pos_ * (1.0 - clamped) + target_pos_ * clamped;
            
            // Solve IK
            KDL::JntArray q_target(6);
            KDL::Frame desired_pose(start_orientation_, target);
            
            int ik_result = ik_solver_->CartToJnt(q_current_, desired_pose, q_target);
            if (ik_result < 0) {
              RCLCPP_WARN(get_logger(), "IK failed, error: %d", ik_result);
              progress_ -= speed_ / total_distance_;
              if (progress_ < 0.0) progress_ = 0.0;
              return;
            }
          
            trajectory_msgs::msg::JointTrajectoryPoint cmd;
            cmd.positions.resize(6);
            cmd.positions[0] = q_target(1);  // shoulder_lift_joint
            cmd.positions[1] = q_target(2);  // elbow_joint
            cmd.positions[2] = q_target(3);  // wrist_1_joint
            cmd.positions[3] = q_target(4);  // wrist_2_joint
            cmd.positions[4] = q_target(5);  // wrist_3_joint
            cmd.positions[5] = q_target(0);  // shoulder_pan_joint
            cmd.velocities.resize(6, 0.0);
          
            pub_->publish(cmd);
            
            q_current_ = q_target;
            progress_ += speed_ / total_distance_;
          });
      });

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/target_joint_positions", 10);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr motion_timer_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  KDL::JntArray q_current_;
  KDL::Vector start_pos_, target_pos_;
  KDL::Rotation start_orientation_;
  
  bool urdf_loaded_ = false;
  bool motion_started_ = false;
  double progress_ = 0.0;
  double speed_ = 0.01;
  double total_distance_ = 0.0;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianMotionController>());
  rclcpp::shutdown();
  return 0;
}