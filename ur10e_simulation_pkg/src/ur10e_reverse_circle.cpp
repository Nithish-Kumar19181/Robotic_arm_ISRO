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

class ReverseCircleForce : public rclcpp::Node
{
public:
  ReverseCircleForce()
  : Node("retract_rotate_approach_circle")
  {
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&ReverseCircleForce::jointCb, this, _1));

    robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&ReverseCircleForce::robotDescCb, this, _1));

    ft_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/ft_sensor_wrench", 10,
      std::bind(&ReverseCircleForce::ftCb, this, _1));

    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_compliance_controller/target_frame", 10);

    target_wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/target_wrench", 10);

    timer_ = create_wall_timer(
      20ms, std::bind(&ReverseCircleForce::update, this));

    RCLCPP_INFO(get_logger(),
      "Retract → Rotate → Approach → Circle → STOP");
  }

private:
  enum class Mode { RETRACT, ROTATE, APPROACH, CIRCLE, STOP };
  Mode mode_{Mode::RETRACT};

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
      radius_ = std::hypot(current_x_, current_y_);
      theta_ = std::atan2(current_y_, current_x_);
      traveled_angle_ = 0.0;
      force_integral_ = 0.0;

      circle_start_time_ = now();
      last_update_time_ = now();

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
    for (size_t i = 0; i < 6; ++i)
      joints_(i) = joint_state_.position[i];

    KDL::Frame frame;
    fk_solver_->JntToCart(joints_, frame);

    start_x_ = frame.p.x();
    start_y_ = frame.p.y();
    start_z_ = frame.p.z();

    current_x_ = start_x_;
    current_y_ = start_y_;

    frame.M.GetQuaternion(qx_, qy_, qz_, qw_);

    double n = std::hypot(start_x_, start_y_);
    dir_x_ = -start_x_ / n;
    dir_y_ = -start_y_ / n;

    retract_start_time_ = now();
  }

  double Scurve(double t)
  {
    t = std::clamp(t, 0.0, 1.0);
    return 3*t*t - 2*t*t*t;
  }

  void publishTargetPose()
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = "base_link";

    if (mode_ == Mode::RETRACT)
    {
      double t = (now() - retract_start_time_).seconds() / retract_duration_;
      double s = Scurve(t);

      current_x_ = start_x_ + s * retract_distance_ * dir_x_;
      current_y_ = start_y_ + s * retract_distance_ * dir_y_;
      double z = start_z_ + s * move_up_distance_;

      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = z;

      if (t >= 1.0)
      {
        qx_start_ = qx_;
        qy_start_ = qy_;
        qz_start_ = qz_;
        qw_start_ = qw_;

        start_x_ = current_x_;
        start_y_ = current_y_;
        start_z_ = z;

        rotate_start_time_ = now();
        mode_ = Mode::ROTATE;
      }
    }

    else if (mode_ == Mode::ROTATE)
    {
      double t = (now() - rotate_start_time_).seconds() / rotate_duration_;
      double s = Scurve(t);

      double angle = s * (-M_PI);

      KDL::Rotation R_start =
          KDL::Rotation::Quaternion(qx_start_, qy_start_, qz_start_, qw_start_);

      KDL::Rotation R_flip =
          KDL::Rotation::RotZ(angle);

      KDL::Rotation R_new = R_start * R_flip;
      R_new.GetQuaternion(qx_, qy_, qz_, qw_);

      pose.pose.position.x = start_x_;
      pose.pose.position.y = start_y_;
      pose.pose.position.z = start_z_;

      if (t >= 1.0)
      {
        approach_start_time_ = now();
        mode_ = Mode::APPROACH;
      }
    }

    else if (mode_ == Mode::APPROACH)
    {
      double t =
        (now() - approach_start_time_).seconds() / approach_duration_;
      t = std::clamp(t, 0.0, 1.0);

      current_x_ = start_x_ - t * approach_distance_ * dir_x_;
      current_y_ = start_y_ - t * approach_distance_ * dir_y_;

      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = start_z_;
    }

    else if (mode_ == Mode::CIRCLE)
    {
      double dt = (now() - last_update_time_).seconds();
      last_update_time_ = now();
      dt = std::clamp(dt, 0.0, 0.1);

      double elapsed = (now() - circle_start_time_).seconds();
      double total_time = max_angle_rad_ / angular_speed_;
      double ramp_time = 0.15 * total_time;

      double alpha = 1.0;

      if (elapsed < ramp_time)
        alpha = Scurve(elapsed / ramp_time);
      else if (elapsed > total_time - ramp_time)
        alpha = Scurve((total_time - elapsed) / ramp_time);

      double omega = angular_speed_ * alpha;

      double force_error = measured_fz_ - desired_circle_force_;
      force_integral_ += force_error * dt;
      force_integral_ =
        std::clamp(force_integral_, -integral_limit_, integral_limit_);

      double radius_dot =
        kp_radius_ * force_error +
        ki_radius_ * force_integral_;

      radius_dot =
        std::clamp(radius_dot, -max_radius_rate_, max_radius_rate_);

      radius_ += radius_dot * dt;

      double dtheta = omega * dt;
      theta_ += dtheta;
      traveled_angle_ += std::abs(dtheta);

      if (traveled_angle_ >= max_angle_rad_)
      {
        mode_ = Mode::STOP;
        RCLCPP_WARN(get_logger(), "Circle complete → STOP");
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

      current_x_ = radius_ * std::cos(theta_);
      current_y_ = radius_ * std::sin(theta_);

      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
      pose.pose.position.z = start_z_;
    }

    else
    {
      pose.pose.position.x = current_x_;
      pose.pose.position.y = current_y_;
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

  rclcpp::Time retract_start_time_, rotate_start_time_, approach_start_time_;
  rclcpp::Time circle_start_time_, last_update_time_;

  double start_x_, start_y_, start_z_;
  double current_x_, current_y_;
  double dir_x_, dir_y_;

  double radius_{0.0}, theta_{0.0}, traveled_angle_{0.0};
  double measured_fz_{0.0}, force_integral_{0.0};

  double qx_, qy_, qz_, qw_;
  double qx_start_, qy_start_, qz_start_, qw_start_;

  const double retract_duration_{3.0};
  const double rotate_duration_{2.0};
  const double retract_distance_{0.15};
  const double move_up_distance_{0.10};

  const double approach_distance_{0.35};
  const double approach_duration_{7.0};

  const double angular_speed_{0.25};
  const double contact_force_threshold_{2.0};
  const double desired_circle_force_{-10.0};

  const double kp_radius_{0.5};
  const double ki_radius_{0.005};
  const double integral_limit_{2.0};
  const double max_radius_rate_{0.01};
  const double max_angle_rad_{7.29};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ReverseCircleForce>());
  rclcpp::shutdown();
  return 0;
}