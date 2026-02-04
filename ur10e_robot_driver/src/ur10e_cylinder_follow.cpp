#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <memory>
#include <algorithm>
#include <cmath>

using namespace std::chrono_literals;
using std::placeholders::_1;

class ApproachThenCircleForce : public rclcpp::Node
{
public:
  ApproachThenCircleForce()
  : Node("approach_then_circle_force")
  {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&ApproachThenCircleForce::jointCb, this, _1));

    robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ApproachThenCircleForce::robotDescCb, this, _1));

    ft_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/ft_sensor_wrench", 10,
      std::bind(&ApproachThenCircleForce::ftCb, this, _1));

    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_compliance_controller/target_frame", 10);

    target_wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/target_wrench", 10);

    timer_ = create_wall_timer(
      20ms, std::bind(&ApproachThenCircleForce::update, this));

    RCLCPP_INFO(get_logger(),
      "Approach → PI force circle (330°) → STOP");
  }

private:
  enum class Mode { APPROACH, CIRCLE, STOP };
  Mode mode_{Mode::APPROACH};

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

  void ftCb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    got_ft_ = true;
    measured_fz_ = msg->wrench.force.z;

    if (mode_ == Mode::APPROACH &&
        std::abs(measured_fz_) > contact_force_threshold_)
    {
      radius_ = std::hypot(current_x_, start_y_);
      start_angle_ = std::atan2(start_y_, current_x_);
      theta_ = start_angle_;

      traveled_angle_ = 0.0;
      force_integral_ = 0.0;

      last_update_time_ = now();
      circle_start_time_ = now();

      mode_ = Mode::CIRCLE;

      RCLCPP_WARN(get_logger(),
        "Contact detected → CIRCLE | Radius = %.4f", radius_);
    }
  }

  void update()
  {
    if (!got_joints_ || !got_urdf_ || !got_ft_)
      return;

    if (!kdl_ready_)
    {
      initKDL();
      readStartPose();
      kdl_ready_ = true;
    }

    publishTargetPose();
    publishTargetWrench();
  }

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

    joints_(0) = joint_state_.position[5];
    joints_(1) = joint_state_.position[0];
    joints_(2) = joint_state_.position[1];
    joints_(3) = joint_state_.position[2];
    joints_(4) = joint_state_.position[3];
    joints_(5) = joint_state_.position[4];

    KDL::Frame frame;
    fk_solver_->JntToCart(joints_, frame);

    start_x_ = frame.p.x();
    start_y_ = frame.p.y();
    start_z_ = frame.p.z();
    current_x_ = start_x_;

    frame.M.GetQuaternion(qx_, qy_, qz_, qw_);
    start_time_ = now();
  }

  void publishTargetPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "base_link";

    if (mode_ == Mode::APPROACH)
    {
      double t =
        (now() - start_time_).seconds() / approach_duration_;
      t = std::clamp(t, 0.0, 1.0);

      current_x_ = start_x_ - t * approach_distance_;

      pose.pose.position.x = current_x_;
      pose.pose.position.y = start_y_;
      pose.pose.position.z = start_z_;
    }
    else if (mode_ == Mode::CIRCLE)
    {
      double dt = (now() - last_update_time_).seconds();
      last_update_time_ = now();
      dt = std::clamp(dt, 0.0, 0.1);

      double elapsed = (now() - circle_start_time_).seconds();
      double total_time = max_angle_rad_ / angular_speed_;
      double ramp_time = accel_ratio_ * total_time;

      double alpha = 1.0;
      if (elapsed < ramp_time)
      {
        double s = elapsed / ramp_time;
        alpha = 3*s*s - 2*s*s*s;
      }
      else if (elapsed > total_time - ramp_time)
      {
        double s = (total_time - elapsed) / ramp_time;
        s = std::clamp(s, 0.0, 1.0);
        alpha = 3*s*s - 2*s*s*s;
      }

      double omega = -angular_speed_ * alpha;

      double force_error = measured_fz_ - desired_circle_force_;
      force_integral_ += force_error * dt;
      force_integral_ =
        std::clamp(force_integral_,
                  -integral_limit_,
                    integral_limit_);

      double radius_dot =
        kp_radius_ * force_error +
        ki_radius_ * force_integral_;

      radius_dot = std::clamp(radius_dot,
                              -max_radius_rate_,
                              max_radius_rate_);

      radius_ += radius_dot * dt;

      double dtheta = omega * dt;
      theta_ += dtheta;
      traveled_angle_ += std::abs(dtheta);

      if (traveled_angle_ >= max_angle_rad_)
      {
        mode_ = Mode::STOP;
        RCLCPP_WARN(get_logger(),
          "Motion complete → STOP (%.1f deg)",
          traveled_angle_ * 180.0 / M_PI);
      }

      KDL::Vector z_axis(std::cos(theta_), std::sin(theta_), 0.0);
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

      pose.pose.position.x = radius_ * std::cos(theta_);
      pose.pose.position.y = radius_ * std::sin(theta_);
      pose.pose.position.z = start_z_;

      pose.pose.orientation.x = qx_;
      pose.pose.orientation.y = qy_;
      pose.pose.orientation.z = qz_;
      pose.pose.orientation.w = qw_;
    }

    else // STOP
    {
      pose.pose.position.x = radius_ * std::cos(theta_);
      pose.pose.position.y = radius_ * std::sin(theta_);
      pose.pose.position.z = start_z_;
    }

    pose.pose.orientation.x = qx_;
    pose.pose.orientation.y = qy_;
    pose.pose.orientation.z = qz_;
    pose.pose.orientation.w = qw_;

    target_pose_pub_->publish(pose);
  }

  void publishTargetWrench()
  {
    geometry_msgs::msg::WrenchStamped wrench;
    wrench.header.stamp = now();
    wrench.header.frame_id = "tool0";

    wrench.wrench.force.z =
      (mode_ == Mode::STOP) ? 0.0 :
      (mode_ == Mode::APPROACH ? -5.0 : -3.0);

    target_wrench_pub_->publish(wrench);
  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr ft_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr target_wrench_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  sensor_msgs::msg::JointState joint_state_;
  std::string robot_description_;

  bool got_joints_{false}, got_urdf_{false}, got_ft_{false}, kdl_ready_{false};

  KDL::Chain chain_;
  KDL::JntArray joints_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  rclcpp::Time start_time_, circle_start_time_, last_update_time_;

  double start_x_, start_y_, start_z_;
  double current_x_;

  double radius_{0.0};
  double start_angle_{0.0};
  double theta_{0.0};
  double traveled_angle_{0.0};

  double measured_fz_{0.0};
  double force_integral_{0.0};

  double qx_, qy_, qz_, qw_;

  const double approach_distance_ = 0.25;
  const double approach_duration_ = 7.0;
  const double angular_speed_ = 0.05;

  const double contact_force_threshold_ = 2.0;
  const double desired_circle_force_ = -10.0;

  const double kp_radius_ = 0.5;
  const double ki_radius_ = 0.005;
  const double integral_limit_ = 2.0;

  const double max_radius_rate_ = 0.01;

  const double max_angle_rad_ = 5.769; // 330 deg
  const double accel_ratio_ = 0.15;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApproachThenCircleForce>());
  rclcpp::shutdown();
  return 0;
}
