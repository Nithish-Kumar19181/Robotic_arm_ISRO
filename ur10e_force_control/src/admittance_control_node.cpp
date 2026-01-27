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
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>

#include <unordered_map>
#include <cmath>
#include <algorithm>

using namespace std::placeholders;
using CallbackReturn =
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class AdmittanceControlNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  AdmittanceControlNode()
  : LifecycleNode("admittance_control_node")
  {
    RCLCPP_INFO(get_logger(), "Admittance Control Node (Always initializes from current joints)");
  }

private:
  /* ================= ROS ================= */

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;

  rclcpp_lifecycle::LifecyclePublisher<
    trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr cmd_pub_;

  rclcpp_lifecycle::LifecyclePublisher<
    std_msgs::msg::Bool>::SharedPtr done_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  /* ================= KDL ================= */

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
  KDL::JntArray q_current_;

  /* ================= State ================= */

  std::unordered_map<std::string, double> joint_map_;
  bool urdf_ready_ = false;
  bool have_joint_state_ = false;
  bool initialized_ = false;

  /* ================= Control ================= */

  double fz_raw_  = 0.0;
  double fz_filt_ = 0.0;
  double alpha_f_ = 0.01;

  double radius_ = 0.0;
  double theta_  = 0.0;
  double start_z_ = 0.0;

  double omega_ = 0.06;
  double F_des_ = -5.0;
  double admittance_gain_ = 0.5e-4;

  double r_min_ = 0.005;
  double r_max_ = 2.0;

  const double DT_ = 1.0 / 500.0;

  double accumulated_angle_ = 0.0;
  bool finished_ = false;

  /* ================= Lifecycle ================= */

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&AdmittanceControlNode::on_urdf, this, _1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      std::bind(&AdmittanceControlNode::on_joint_state, this, _1));

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench", rclcpp::QoS(10),
      std::bind(&AdmittanceControlNode::on_wrench, this, _1));

    cmd_pub_ = create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
      "/target_joint_positions", 10);

    done_pub_ = create_publisher<std_msgs::msg::Bool>(
      "/admittance_done", rclcpp::QoS(1).transient_local());

    RCLCPP_INFO(get_logger(), "Admittance node configured");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    cmd_pub_->on_activate();
    done_pub_->on_activate();

    accumulated_angle_ = 0.0;
    finished_ = false;
    initialized_ = false;

    // Try initialize immediately if joint state already available
    try_initialize_from_current();

    RCLCPP_WARN(get_logger(), "Admittance control ACTIVATED");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    if (timer_)
      timer_->cancel();

    cmd_pub_->on_deactivate();
    done_pub_->on_deactivate();

    RCLCPP_WARN(get_logger(), "Admittance control DEACTIVATED by manager");
    return CallbackReturn::SUCCESS;
  }

  /* ================= Callbacks ================= */

  void on_wrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    fz_raw_ = msg->wrench.force.z;
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

    // FK
    KDL::Frame pose;
    fk_solver_->JntToCart(q_current_, pose);

    radius_  = std::hypot(pose.p.x(), pose.p.y());
    theta_   = std::atan2(pose.p.y(), pose.p.x());
    start_z_ = pose.p.z();

    accumulated_angle_ = 0.0;
    finished_ = false;
    initialized_ = true;

    RCLCPP_INFO(get_logger(), "Admittance initialized from CURRENT joints: R=%.3f theta=%.3f",
      radius_, theta_);

    // Start control loop
    timer_ = create_wall_timer(
      std::chrono::duration<double>(DT_),
      std::bind(&AdmittanceControlNode::step, this));
  }

  /* ================= Control Loop ================= */

  void step()
  {
    if (!initialized_ || finished_)
      return;

    // Filter force
    fz_filt_ = alpha_f_ * fz_raw_ + (1.0 - alpha_f_) * fz_filt_;

    // Admittance radius control
    double force_error = fz_filt_ - F_des_;
    radius_ += admittance_gain_ * force_error * DT_;
    radius_ = std::clamp(radius_, r_min_, r_max_);

    // Advance angle
    double dtheta = omega_ * DT_;
    theta_ += dtheta;
    accumulated_angle_ += std::abs(dtheta);

    if (accumulated_angle_ >= 2.0 * M_PI)
    {
      std_msgs::msg::Bool msg;
      msg.data = true;
      done_pub_->publish(msg);

      finished_ = true;
      RCLCPP_WARN(get_logger(), "Admittance 360° COMPLETED");
      return;
    }

    // Desired position
    KDL::Vector normal(std::cos(theta_), std::sin(theta_), 0.0);

    KDL::Vector p(
      radius_ * normal.x(),
      radius_ * normal.y(),
      start_z_
    );

    // Orientation
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

    KDL::Frame desired_pose(R, p);

    // IK
    KDL::JntArray q_sol = q_current_;

    if (ik_solver_->CartToJnt(q_current_, desired_pose, q_sol) < 0)
      return;

    trajectory_msgs::msg::JointTrajectoryPoint pt;
    pt.positions.resize(q_sol.rows());
    for (unsigned int i = 0; i < q_sol.rows(); ++i)
      pt.positions[i] = q_sol(i);

    cmd_pub_->publish(pt);
    q_current_ = q_sol;
  }
};

/* ================= main ================= */

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<AdmittanceControlNode>();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
