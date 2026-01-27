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

using namespace std::placeholders;

class CartesianStraightLineSimple : public rclcpp::Node
{
public:
  CartesianStraightLineSimple()
  : Node("cartesian_straight_line_simple")
  {
    RCLCPP_INFO(get_logger(), "Waiting for URDF...");

    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&CartesianStraightLineSimple::on_urdf, this, _1));

    pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/target_joint_positions", 10);
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
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
  double speed_ = 0.002;   // meters per tick
  double total_distance_ = 0.0;

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (ik_solver_) return;

    // --- Parse URDF ---
    urdf::Model model;
    if (!model.initString(msg->data))
    {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF");
      return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree))
    {
      RCLCPP_ERROR(get_logger(), "Failed to create KDL tree");
      return;
    }

    if (!tree.getChain("base_link", "tool0", chain_))
    {
      RCLCPP_ERROR(get_logger(), "Failed to get chain");
      return;
    }

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

    target_pos_ = start_pos_ - KDL::Vector(0.20, 0.0, 0.0);

    total_distance_ = (target_pos_ - start_pos_).Norm();

    RCLCPP_INFO(get_logger(), "Moving in straight line %.3f meters in -X", total_distance_);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(30),
      std::bind(&CartesianStraightLineSimple::step, this));
  }

  void step()
  {
    if (progress_ >= 1.0)
    {
      RCLCPP_INFO(get_logger(), "Cartesian motion completed");
      timer_->cancel();
      return;
    }

    // Linear interpolation parameter
    progress_ += speed_ / total_distance_;
    if (progress_ > 1.0) progress_ = 1.0;

    // Linear interpolation in Cartesian space
    KDL::Vector p = start_pos_ * (1.0 - progress_) + target_pos_ * progress_;

    KDL::Frame desired_pose(fixed_orientation_, p);

    KDL::JntArray q_sol(chain_.getNrOfJoints());
    int ret = ik_solver_->CartToJnt(q_current_, desired_pose, q_sol);
    if (ret < 0)
    {
      RCLCPP_WARN(get_logger(), "IK failed at progress=%.3f", progress_);
      return;
    }

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
