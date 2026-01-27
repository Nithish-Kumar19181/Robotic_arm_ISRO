#include "rclcpp/rclcpp.hpp"

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <deque>

using namespace std::placeholders;

class CartesianStraightLineSimple : public rclcpp::Node
{
public:
  CartesianStraightLineSimple()
  : Node("cartesian_straight_line_simple")
  {
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&CartesianStraightLineSimple::on_urdf, this, _1));

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/target_joint_positions", 10);

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench", rclcpp::QoS(500),
      std::bind(&CartesianStraightLineSimple::on_wrench, this, _1));
      
    // Add joint state subscription to get current position
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&CartesianStraightLineSimple::on_joint_state, this, _1));
  }

private:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::JntArray q_current_;  // In KDL order

  KDL::Vector start_pos_;
  KDL::Vector target_pos_;
  KDL::Rotation fixed_orientation_;

  double progress_ = 0.0;
  double speed_ = 0.00002;  
  double total_distance_ = 0.0;

  double fx_ = 0.0;

  struct Sample { double x; double fx; };
  std::deque<Sample> window_;

  const size_t WINDOW_SIZE_ = 5;       
  const double STIFFNESS_THRESH_ = 300; 
  const double RETRACT_DIST_ = 0.1;

  enum class MotionState { FORWARD, RETRACT, DONE };
  MotionState state_ = MotionState::FORWARD;
  
  bool urdf_loaded_ = false;
  bool initialized_ = false;

  void on_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    fx_ = msg->wrench.force.x;
  }
  
  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (!urdf_loaded_ || initialized_) return;
    
    q_current_(0) = msg->position[5];  // shoulder_pan_joint
    q_current_(1) = msg->position[0];  // shoulder_lift_joint
    q_current_(2) = msg->position[1];  // elbow_joint
    q_current_(3) = msg->position[2];  // wrist_1_joint
    q_current_(4) = msg->position[3];  // wrist_2_joint
    q_current_(5) = msg->position[4];  // wrist_3_joint
    
    KDL::Frame start_pose;
    if (fk_solver_->JntToCart(q_current_, start_pose) < 0) return;
    
    start_pos_ = start_pose.p;
    fixed_orientation_ = start_pose.M;
    
    target_pos_ = start_pos_ - KDL::Vector(0.20, 0, 0);
    total_distance_ = (target_pos_ - start_pos_).Norm();
    
    window_.clear();
    initialized_ = true;
    
    timer_ = create_wall_timer(
      std::chrono::microseconds(2000),   // 2 ms = 500 Hz
      std::bind(&CartesianStraightLineSimple::step, this));
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (urdf_loaded_) return;

    urdf::Model model;
    model.initString(msg->data);

    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(model, tree);
    tree.getChain("base_link", "tool0", chain_);

    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

    q_current_ = KDL::JntArray(chain_.getNrOfJoints());
    
    urdf_loaded_ = true;
  }

  void step()
  {
    if (state_ == MotionState::DONE)
    {
      timer_->cancel();
      return;
    }
    
    if (!initialized_) return;

    double curr_x =
      (start_pos_ * (1.0 - progress_) +
       target_pos_ * progress_).x();

    window_.push_back({curr_x, fx_});
    if (window_.size() > WINDOW_SIZE_)
      window_.pop_front();

    if (state_ == MotionState::FORWARD && window_.size() == WINDOW_SIZE_)
    {
      const auto& s0 = window_.front();
      const auto& sN = window_.back();

      double dx = sN.x - s0.x;
      double dF = sN.fx - s0.fx;

      if (dx < 0.0)
      {
        double k_avg = dF / dx;

        if (k_avg < -STIFFNESS_THRESH_ && sN.fx < 0.0)
        {
          KDL::Vector curr_pos =
            start_pos_ * (1.0 - progress_) +
            target_pos_ * progress_;

          start_pos_ = curr_pos;
          target_pos_ = start_pos_ + KDL::Vector(RETRACT_DIST_, 0, 0);

          total_distance_ = (target_pos_ - start_pos_).Norm();
          progress_ = 0.0;
          state_ = MotionState::RETRACT;
          return;
        }
      }
    }

    if (progress_ >= 1.0)
    {
      if (state_ == MotionState::RETRACT)
        state_ = MotionState::DONE;
      return;
    }
    
    progress_ += speed_ / total_distance_;
    if (progress_ > 1.0) progress_ = 1.0;

    KDL::Vector p =
      start_pos_ * (1.0 - progress_) +
      target_pos_ * progress_;

    KDL::Frame desired_pose(fixed_orientation_, p);

    KDL::JntArray q_sol(chain_.getNrOfJoints());
    if (ik_solver_->CartToJnt(q_current_, desired_pose, q_sol) < 0)
      return;

    send_joint_command(q_sol);
    
    q_current_ = q_sol;
  }
  
  void send_joint_command(const KDL::JntArray& joint_positions)
  {
    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(6);
    
    pt.positions[0] = joint_positions(1);  // shoulder_lift_joint
    pt.positions[1] = joint_positions(2);  // elbow_joint
    pt.positions[2] = joint_positions(3);  // wrist_1_joint
    pt.positions[3] = joint_positions(4);  // wrist_2_joint
    pt.positions[4] = joint_positions(5);  // wrist_3_joint
    pt.positions[5] = joint_positions(0);  // shoulder_pan_joint
    
    pub_->publish(pt);
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianStraightLineSimple>());
  rclcpp::shutdown();
  return 0;
}