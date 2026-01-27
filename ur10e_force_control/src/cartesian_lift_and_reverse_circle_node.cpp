#include "rclcpp/rclcpp.hpp"
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unordered_map>
#include <cmath>

using namespace std::placeholders;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class CartesianLiftAndReverseCircle : public rclcpp_lifecycle::LifecycleNode
{
public:
  CartesianLiftAndReverseCircle()
  : LifecycleNode("cartesian_lift_and_reverse_circle_node")
  {
    RCLCPP_INFO(get_logger(), "Cartesian Lift + Reverse Circle (Always initializes from current joints, 500Hz)");
  }

private:

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp_lifecycle::LifecyclePublisher<
    trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr cmd_pub_;

  rclcpp_lifecycle::LifecyclePublisher<
    std_msgs::msg::Bool>::SharedPtr done_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  KDL::JntArray q_current_;

  std::unordered_map<std::string, double> joint_map_;
  bool urdf_ready_ = false;
  bool have_joint_state_ = false;
  bool initialized_ = false;

  int stage_ = 0;   // 0:+X, 1:+Z, 2:reverse circle

  // Cartesian linear motion
  KDL::Vector start_pos_;
  KDL::Rotation fixed_ori_;
  double lin_progress_ = 0.0;

  double lin_dist_x_ = 0.10;   // 10 cm X
  double lin_dist_z_ = 0.10;   // 10 cm Z

  double lin_speed_x_ = 0.001;    // fast X
  double lin_speed_z_ = 0.0005;   // slow Z

  // Circle motion
  double radius_ = 0.0;
  double theta_ = 0.0;
  double start_z_ = 0.0;
  double omega_ = -0.2;   // reverse direction
  double accumulated_angle_ = 0.0;

  const double DT_ = 1.0 / 500.0;   // EXACT 500 Hz

  bool finished_ = false;

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&CartesianLiftAndReverseCircle::on_urdf, this, _1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      std::bind(&CartesianLiftAndReverseCircle::on_joint_state, this, _1));

    cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/target_joint_positions", 10);

    done_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/reverse_motion_done", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(get_logger(), "Cartesian lift node configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    cmd_pub_->on_activate();
    done_pub_->on_activate();
    stage_ = 0;
    lin_progress_ = 0.0;
    accumulated_angle_ = 0.0;
    finished_ = false;
    initialized_ = false;

    try_initialize_from_current();

    RCLCPP_WARN(get_logger(), "Cartesian lift + reverse circle ACTIVATED");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (timer_) timer_->cancel();
    cmd_pub_->on_deactivate();
    done_pub_->on_deactivate();

    RCLCPP_WARN(get_logger(), "Cartesian lift node DEACTIVATED by manager");
    return CallbackReturn::SUCCESS;
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
    ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(chain_);
    q_current_ = KDL::JntArray(chain_.getNrOfJoints());

    urdf_ready_ = true;

    try_initialize_from_current();
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_map_[msg->name[i]] = msg->position[i];

    have_joint_state_ = true;

    if (get_current_state().id() ==
        lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
    {
      if (!initialized_)
        try_initialize_from_current();
    }
  }

  void try_initialize_from_current()
  {
    if (initialized_ || !urdf_ready_ || !have_joint_state_)
      return;

    unsigned int idx = 0;
    for (const auto &seg : chain_.segments)
    {
      if (seg.getJoint().getType() != KDL::Joint::None)
      {
        const std::string &name = seg.getJoint().getName();
        if (!joint_map_.count(name))
          return;
        q_current_(idx++) = joint_map_[name];
      }
    }

    KDL::Frame pose;
    fk_solver_->JntToCart(q_current_, pose);

    start_pos_ = pose.p;
    fixed_ori_ = pose.M;

    // Circle params
    radius_ = std::hypot(start_pos_.x(), start_pos_.y());
    theta_  = std::atan2(start_pos_.y(), start_pos_.x());
    start_z_ = start_pos_.z();

    stage_ = 0;
    lin_progress_ = 0.0;
    accumulated_angle_ = 0.0;
    finished_ = false;
    initialized_ = true;

    RCLCPP_INFO(get_logger(), "Cartesian lift initialized from CURRENT joints");

    // Start control loop
    timer_ = create_wall_timer(
      std::chrono::duration<double>(DT_),
      std::bind(&CartesianLiftAndReverseCircle::step, this));
  }

  void step()
  {
    if (!initialized_ || finished_)
      return;

    KDL::Frame desired;

    if (stage_ == 0)
    {
      // +X motion
      lin_progress_ += lin_speed_x_;
      if (lin_progress_ > lin_dist_x_) lin_progress_ = lin_dist_x_;

      KDL::Vector p = start_pos_ + KDL::Vector(lin_progress_, 0, 0);
      desired = KDL::Frame(fixed_ori_, p);

      if (lin_progress_ >= lin_dist_x_)
      {
        stage_ = 1;
        start_pos_ = p;
        lin_progress_ = 0.0;
      }
    }
    else if (stage_ == 1)
    {
      lin_progress_ += lin_speed_z_;
      if (lin_progress_ > lin_dist_z_) lin_progress_ = lin_dist_z_;

      KDL::Vector p = start_pos_ + KDL::Vector(0, 0, lin_progress_);
      desired = KDL::Frame(fixed_ori_, p);

      if (lin_progress_ >= lin_dist_z_)
      {
        stage_ = 2;
        accumulated_angle_ = 0.0;

        radius_ = std::hypot(p.x(), p.y());
        theta_  = std::atan2(p.y(), p.x());
        start_z_ = p.z();
      }
    }
    else if (stage_ == 2)
    {

      double dtheta = omega_ * DT_;
      theta_ += dtheta;
      accumulated_angle_ += std::abs(dtheta);

      KDL::Vector normal(std::cos(theta_), std::sin(theta_), 0.0);

      KDL::Vector p(
        radius_ * normal.x(),
        radius_ * normal.y(),
        start_z_
      );

      KDL::Vector z_axis = normal;
      KDL::Vector y_axis(0.0, 0.0, 1.0);
      KDL::Vector x_axis = y_axis * z_axis;

      x_axis.Normalize();
      y_axis = z_axis * x_axis;
      y_axis.Normalize();

      KDL::Rotation R(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z()
      );

      desired = KDL::Frame(R, p);

      if (accumulated_angle_ >= 2.0 * M_PI)
      {
        finished_ = true;

        std_msgs::msg::Bool msg;
        msg.data = true;
        done_pub_->publish(msg);

        RCLCPP_WARN(get_logger(), "Reverse circle completed");
        return;
      }
    }
    else
    {
      return;
    }

    // IK
    KDL::JntArray q_sol = q_current_;
    if (ik_solver_->CartToJnt(q_current_, desired, q_sol) < 0)
      return;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(q_sol.rows());
    for (unsigned int i = 0; i < q_sol.rows(); ++i)
      pt.positions[i] = q_sol(i);

    cmd_pub_->publish(pt);
    q_current_ = q_sol;
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CartesianLiftAndReverseCircle>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
