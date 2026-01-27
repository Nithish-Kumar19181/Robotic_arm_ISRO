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
  }

private:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::JntArray q_current_;

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

  void on_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    fx_ = msg->wrench.force.x;
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (ik_solver_) return;

    urdf::Model model;
    model.initString(msg->data);

    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(model, tree);
    tree.getChain("base_link", "tool0", chain_);

    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);

    q_current_ = KDL::JntArray(chain_.getNrOfJoints());

    q_current_(0) = -3.613240;
    q_current_(1) = -1.662099;
    q_current_(2) =  2.463900;
    q_current_(3) = -3.943181;
    q_current_(4) =  5.184341;
    q_current_(5) = -1.571551;

    KDL::Frame start_pose;
    fk_solver_->JntToCart(q_current_, start_pose);

    start_pos_ = start_pose.p;
    fixed_orientation_ = start_pose.M;

    target_pos_ = start_pos_ - KDL::Vector(0.20, 0, 0);
    total_distance_ = (target_pos_ - start_pos_).Norm();

    window_.clear();

    // 500 Hz TIMER 
    timer_ = create_wall_timer(
      std::chrono::microseconds(2000),   // 2 ms = 500 Hz
      std::bind(&CartesianStraightLineSimple::step, this));
  }

  void step()
  {
    if (state_ == MotionState::DONE)
    {
      timer_->cancel();
      return;
    }

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
          RCLCPP_WARN(
            get_logger(),
            "Wall detected @500Hz | k=%.1f N/m | Fx=%.2f N",
            k_avg, sN.fx);

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

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(q_sol.rows());
    for (unsigned int i = 0; i < q_sol.rows(); ++i)
      pt.positions[i] = q_sol(i);

    pub_->publish(pt);
    q_current_ = q_sol;
  }
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianStraightLineSimple>());
  rclcpp::shutdown();
  return 0;
}
