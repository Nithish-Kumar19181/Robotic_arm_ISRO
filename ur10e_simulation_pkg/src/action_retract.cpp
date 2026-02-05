#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include "robot_task_interfaces/action/execute_skill.hpp"

#include <memory>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

using ExecuteSkill = robot_task_interfaces::action::ExecuteSkill;
using GoalHandleExecuteSkill =
  rclcpp_action::ServerGoalHandle<ExecuteSkill>;

class MoveToOriginThenHelix : public rclcpp::Node
{
public:
  MoveToOriginThenHelix()
  : Node("move_to_origin_then_helix")
  {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&MoveToOriginThenHelix::jointCb, this, _1));

    robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&MoveToOriginThenHelix::robotDescCb, this, _1));

    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_compliance_controller/target_frame", 10);

    timer_ = create_wall_timer(
      20ms, std::bind(&MoveToOriginThenHelix::update, this));

    action_server_ =
      rclcpp_action::create_server<ExecuteSkill>(
        this,
        "/move_to_origin_then_helix",
        std::bind(&MoveToOriginThenHelix::handleGoal, this, _1, _2),
        std::bind(&MoveToOriginThenHelix::handleCancel, this, _1),
        std::bind(&MoveToOriginThenHelix::handleAccepted, this, _1));

    RCLCPP_INFO(get_logger(),
      "Action Server ready: S-curve Move → S-curve Helix → STOP");
  }

private:
  /* ================= ACTION ================= */

  rclcpp_action::Server<ExecuteSkill>::SharedPtr action_server_;
  std::shared_ptr<GoalHandleExecuteSkill> active_goal_;

  bool running_{false};
  bool result_sent_{false};

  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<const ExecuteSkill::Goal>)
  {
    return running_
      ? rclcpp_action::GoalResponse::REJECT
      : rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandleExecuteSkill> goal_handle)
  {
    mode_ = Mode::STOP;
    running_ = false;

    auto result = std::make_shared<ExecuteSkill::Result>();
    result->success = false;
    result->message = "Helix canceled";

    goal_handle->canceled(result);
    active_goal_.reset();

    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handleAccepted(
    const std::shared_ptr<GoalHandleExecuteSkill> goal_handle)
  {
    active_goal_ = goal_handle;
    resetMotion();
    running_ = true;
    result_sent_ = false;

    RCLCPP_INFO(get_logger(), "Helix action started");
  }

  /* ================= MOTION ================= */

  enum class Mode { MOVE, HELIX, STOP };
  Mode mode_{Mode::STOP};

  void resetMotion()
  {
    kdl_ready_ = false;
    mode_ = Mode::MOVE;
  }

  /* ================= CALLBACKS ================= */

  void jointCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_state_ = *msg;
    got_joints_ = true;
  }

  void robotDescCb(const std_msgs::msg::String::SharedPtr msg)
  {
    robot_description_ = msg->data;
    got_urdf_ = true;
  }

  /* ================= UPDATE LOOP ================= */

  void update()
  {
    if (!running_ || !got_joints_ || !got_urdf_)
      return;

    if (!kdl_ready_)
    {
      initKDL();
      readStartPose();
      kdl_ready_ = true;
    }

    publishTargetPose();
    publishFeedbackAndResult();
  }

  /* ================= SCURVE ================= */

  double sCurve(double t)
  {
    t = std::clamp(t, 0.0, 1.0);
    return 3*t*t - 2*t*t*t;
  }

  /* ================= KDL ================= */

  void initKDL()
  {
    urdf::Model model;
    model.initString(robot_description_);

    KDL::Tree tree;
    kdl_parser::treeFromUrdfModel(model, tree);
    tree.getChain("base_link", "tool0", chain_);

    fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);
    joints_.resize(chain_.getNrOfJoints());
  }

  void readStartPose()
  {
    for (size_t i = 0; i < 6; ++i)
      joints_(i) = joint_state_.position[i];

    KDL::Frame frame;
    fk_solver_->JntToCart(joints_, frame);

    start_x_ = frame.p.x();
    start_y_ = frame.p.y();
    start_z_ = frame.p.z();

    frame.M.GetQuaternion(qx_, qy_, qz_, qw_);

    double n = std::hypot(start_x_, start_y_);
    dir_x_ = -start_x_ / n;
    dir_y_ = -start_y_ / n;

    move_start_time_ = now();
  }

  /* ================= ORIENTATION ================= */

  void computeCircleOrientation(double theta)
  {
    KDL::Vector z_axis(std::cos(theta), std::sin(theta), 0.0);
    z_axis.Normalize();

    KDL::Vector up(0, 0, 1);
    KDL::Vector x_axis = up * z_axis;
    if (x_axis.Norm() < 1e-6)
      x_axis = KDL::Vector(1, 0, 0);
    x_axis.Normalize();

    KDL::Vector y_axis = z_axis * x_axis;
    y_axis.Normalize();

    KDL::Rotation R(x_axis, y_axis, z_axis);
    R.GetQuaternion(qx_, qy_, qz_, qw_);
  }

  /* ================= TRAJECTORY ================= */

  void publishTargetPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "base_link";

    /* ---------- MOVE (S-CURVE) ---------- */
    if (mode_ == Mode::MOVE)
    {
      double t =
        (now() - move_start_time_).seconds() / move_duration_;
      double s = sCurve(t);

      double x = start_x_ + s * move_distance_ * dir_x_;
      double y = start_y_ + s * move_distance_ * dir_y_;

      pose.pose.position.x = x;
      pose.pose.position.y = y;
      pose.pose.position.z = start_z_;

      pose.pose.orientation.x = qx_;
      pose.pose.orientation.y = qy_;
      pose.pose.orientation.z = qz_;
      pose.pose.orientation.w = qw_;

      if (t >= 1.0)
      {
        radius_ = std::hypot(x, y);
        theta_ = std::atan2(y, x);
        theta_start_ = theta_;
        z_start_ = start_z_;

        helix_start_time_ = now();
        last_update_time_ = now();
        mode_ = Mode::HELIX;
      }
    }

    /* ---------- HELIX (S-CURVE ANGULAR) ---------- */
    else if (mode_ == Mode::HELIX)
    {
      double dt = std::clamp(
        (now() - last_update_time_).seconds(), 0.0, 0.1);
      last_update_time_ = now();

      double elapsed = (now() - helix_start_time_).seconds();
      double total_time = max_angle_rad_ / angular_speed_;
      double ramp_time = accel_ratio_ * total_time;

      double alpha = 1.0;
      if (elapsed < ramp_time)
        alpha = sCurve(elapsed / ramp_time);
      else if (elapsed > total_time - ramp_time)
        alpha = sCurve((total_time - elapsed) / ramp_time);

      theta_ += angular_speed_ * alpha * dt;

      double angle_travelled =
        std::abs(theta_ - theta_start_);

      double s =
        std::clamp(angle_travelled / max_angle_rad_, 0.0, 1.0);

      pose.pose.position.x = radius_ * std::cos(theta_);
      pose.pose.position.y = radius_ * std::sin(theta_);
      pose.pose.position.z = z_start_ + s * helix_height_;

      computeCircleOrientation(theta_);

      pose.pose.orientation.x = qx_;
      pose.pose.orientation.y = qy_;
      pose.pose.orientation.z = qz_;
      pose.pose.orientation.w = qw_;

      /* ---------- ROBUST STOP ---------- */
      if (angle_travelled >= max_angle_rad_ ||
          elapsed >= total_time)
      {
        mode_ = Mode::STOP;

        RCLCPP_WARN(get_logger(),
          "HELIX DONE → STOP (%.1f deg)",
          angle_travelled * 180.0 / M_PI);
      }
    }

    target_pose_pub_->publish(pose);
  }

  /* ================= ACTION RESULT ================= */

  void publishFeedbackAndResult()
  {
    if (!active_goal_ || result_sent_)
      return;

    auto feedback =
      std::make_shared<ExecuteSkill::Feedback>();

    feedback->state =
      (mode_ == Mode::MOVE)  ? "MOVE"  :
      (mode_ == Mode::HELIX) ? "HELIX" :
                               "STOP";

    feedback->progress =
      (mode_ == Mode::HELIX)
        ? std::clamp(
            std::abs(theta_ - theta_start_) / max_angle_rad_,
            0.0, 1.0)
        : 0.0;

    active_goal_->publish_feedback(feedback);

    if (mode_ == Mode::STOP)
    {
      auto result =
        std::make_shared<ExecuteSkill::Result>();

      result->success = true;
      result->message = "Helix completed";

      active_goal_->succeed(result);
      active_goal_.reset();
      running_ = false;
      result_sent_ = true;

      RCLCPP_INFO(get_logger(), "Helix → SUCCEEDED");
    }
  }

  /* ================= MEMBERS ================= */

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState joint_state_;
  std::string robot_description_;

  bool got_joints_{false}, got_urdf_{false}, kdl_ready_{false};

  KDL::Chain chain_;
  KDL::JntArray joints_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  rclcpp::Time move_start_time_, helix_start_time_, last_update_time_;

  double start_x_, start_y_, start_z_;
  double dir_x_, dir_y_;
  double radius_{0.0};
  double theta_{0.0}, theta_start_{0.0};
  double z_start_{0.0};

  double qx_, qy_, qz_, qw_;

  const double move_distance_ = 0.15;
  const double move_duration_ = 6.0;

  const double angular_speed_ = 0.4;
  const double max_angle_rad_ = 5.769; // 330°
  const double accel_ratio_ = 0.15;
  const double helix_height_ = 0.10;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToOriginThenHelix>());
  rclcpp::shutdown();
  return 0;
}
