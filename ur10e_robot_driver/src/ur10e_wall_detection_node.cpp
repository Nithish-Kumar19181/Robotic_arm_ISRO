#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unordered_map>
#include <cmath>

using namespace std::placeholders;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class WallDetectionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  WallDetectionNode()
  : LifecycleNode("wall_detection_node")
  {
    RCLCPP_INFO(get_logger(), "Wall Detection Node (Driver joint order fixed)");
  }

private:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

  rclcpp_lifecycle::LifecyclePublisher<
    trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr joint_pub_;

  rclcpp_lifecycle::LifecyclePublisher<
    std_msgs::msg::Bool>::SharedPtr wall_found_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_NR> ik_pos_solver_;
  KDL::JntArray q_current_;

  std::unordered_map<std::string, double> joint_state_map_;
  bool urdf_ready_ = false;
  bool have_joint_state_ = false;
  bool initialized_ = false;

  KDL::Vector start_pos_;
  KDL::Rotation fixed_orientation_;

  double traveled_ = 0.0;
  bool contact_detected_ = false;

  const double SPEED_ = 0.002;
  const double DT_    = 1.0 / 500.0;
  const double MAX_DIST_ = 0.20;

  double fz_raw_ = 0.0;
  double fz_filt_ = 0.0;
  double prev_fz_filt_ = 0.0;

  const double LPF_ALPHA_ = 0.2;
  const double DFDT_THRESH_ = 5.0;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&WallDetectionNode::on_urdf, this, _1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::QoS(50),
      std::bind(&WallDetectionNode::on_joint_state, this, _1));

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench", rclcpp::QoS(50),
      std::bind(&WallDetectionNode::on_wrench, this, _1));

    joint_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/target_joint_positions", 10);

    wall_found_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/wall_found", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(get_logger(), "Wall detection configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    joint_pub_->on_activate();
    wall_found_pub_->on_activate();

    traveled_ = 0.0;
    contact_detected_ = false;
    initialized_ = false;

    prev_fz_filt_ = fz_raw_;
    fz_filt_ = fz_raw_;

    try_initialize_from_current();

    RCLCPP_WARN(get_logger(), "Wall detection ACTIVATED");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (timer_)
      timer_->cancel();

    joint_pub_->on_deactivate();
    wall_found_pub_->on_deactivate();

    RCLCPP_WARN(get_logger(), "Wall detection DEACTIVATED");
    return CallbackReturn::SUCCESS;
  }

  void on_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    fz_raw_ = msg->wrench.force.z;
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_state_map_[msg->name[i]] = msg->position[i];

    have_joint_state_ = true;

    if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      if (!initialized_)
        try_initialize_from_current();
    }
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (urdf_ready_) return;

    urdf::Model model;
    if (!model.initString(msg->data)) return;

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) return;

    if (!tree.getChain("base_link", "tool0", chain_)) return;

    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    ik_vel_solver_ = std::make_shared<KDL::ChainIkSolverVel_pinv>(chain_);
    ik_pos_solver_ = std::make_shared<KDL::ChainIkSolverPos_NR>(
        chain_, *fk_solver_, *ik_vel_solver_, 100, 1e-6);

    q_current_ = KDL::JntArray(6);

    urdf_ready_ = true;

    try_initialize_from_current();
  }

  void try_initialize_from_current()
  {
    if (initialized_ || !urdf_ready_ || !have_joint_state_)
      return;

    if (joint_state_map_.size() < 6)
      return;

    // EXACT SAME MAPPING AS YOUR WORKING NODE
    q_current_(0) = joint_state_map_["shoulder_pan_joint"];
    q_current_(1) = joint_state_map_["shoulder_lift_joint"];
    q_current_(2) = joint_state_map_["elbow_joint"];
    q_current_(3) = joint_state_map_["wrist_1_joint"];
    q_current_(4) = joint_state_map_["wrist_2_joint"];
    q_current_(5) = joint_state_map_["wrist_3_joint"];

    KDL::Frame start_pose;
    fk_solver_->JntToCart(q_current_, start_pose);

    start_pos_ = start_pose.p;
    fixed_orientation_ = start_pose.M;

    traveled_ = 0.0;
    initialized_ = true;

    RCLCPP_INFO(get_logger(), "Initialized at TCP: %.3f %.3f %.3f",
      start_pos_.x(), start_pos_.y(), start_pos_.z());

    timer_ = create_wall_timer(
      std::chrono::duration<double>(DT_),
      std::bind(&WallDetectionNode::step, this));
  }

  void step()
  {
    if (!initialized_ || contact_detected_)
      return;

    // Force filter
    fz_filt_ = LPF_ALPHA_ * fz_raw_
             + (1.0 - LPF_ALPHA_) * fz_filt_;

    double dFdt = (fz_filt_ - prev_fz_filt_) / DT_;
    prev_fz_filt_ = fz_filt_;

    if (std::abs(dFdt) > DFDT_THRESH_ && fz_filt_ < 0.0)
    {
      contact_detected_ = true;

      std_msgs::msg::Bool msg;
      msg.data = true;
      wall_found_pub_->publish(msg);

      RCLCPP_WARN(get_logger(), "CONTACT DETECTED");
      return;
    }

    traveled_ += SPEED_ * DT_;
    if (traveled_ > MAX_DIST_)
      traveled_ = MAX_DIST_;

    KDL::Vector p = start_pos_ - KDL::Vector(traveled_, 0, 0);
    KDL::Frame desired_pose(fixed_orientation_, p);

    KDL::JntArray q_sol = q_current_;

    if (ik_pos_solver_->CartToJnt(q_current_, desired_pose, q_sol) < 0)
      return;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(6);

    // REMAP BACK TO DRIVER ORDER
    pt.positions[0] = q_sol(1);  // shoulder_lift
    pt.positions[1] = q_sol(2);  // elbow
    pt.positions[2] = q_sol(3);  // wrist_1
    pt.positions[3] = q_sol(4);  // wrist_2
    pt.positions[4] = q_sol(5);  // wrist_3
    pt.positions[5] = q_sol(0);  // shoulder_pan

    joint_pub_->publish(pt);
    q_current_ = q_sol;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WallDetectionNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
