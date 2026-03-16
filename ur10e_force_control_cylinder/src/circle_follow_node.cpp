#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "ur10e_force_control_base/robot_state.hpp"
#include "ur10e_force_control_cylinder/primitives/circle_primitive.hpp"

using namespace std::placeholders;
using namespace motion_primitives;

class CircleNode : public rclcpp::Node {
public:
  CircleNode() : Node("circle_node") {
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/ft_sensor_wrench",
      rclcpp::SensorDataQoS(),
      std::bind(&CircleNode::wrenchCallback, this, _1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 
      rclcpp::SensorDataQoS(),
      std::bind(&CircleNode::jointCallback, this, _1));

    robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description", 
      rclcpp::QoS(1).transient_local(),
      std::bind(&CircleNode::robotDescriptionCallback, this, _1));

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_compliance_controller/target_frame", 10);

    wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/cartesian_compliance_controller/target_wrench", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(10),
      std::bind(&CircleNode::timerCallback, this));

    // Configure circle parameters
    circle_.setParameters(0.25,     // angular speed rad/s
                          2 * M_PI, // full circle
                          -20.0,    // target force
                          -1        // direction (CW)
    );
  }

private:
  // ROS
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  RobotState robot_state_;
  CirclePrimitive circle_;

  std::string urdf_string_;

  bool got_joints_{false};
  bool got_wrench_{false};
  bool got_urdf_{false};
  bool started_{false};

  rclcpp::Time last_time_;

private:
  void wrenchCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) 
  {
    robot_state_.wrench_z_ = *msg;
    got_wrench_ = true;
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg) 
  {
    robot_state_.joint_state = *msg;
    got_joints_ = true;
  }

  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg) 
  {
    urdf_string_ = msg->data;
    got_urdf_ = true;
  }

  void timerCallback() 
  {
    if (!got_joints_ || !got_wrench_ || !got_urdf_)
      return;

    rclcpp::Time now = this->now();

    if (!started_) {

        circle_.setForceGains(0.5, 0.001, 2.0, 0.01);

      circle_.onStart(robot_state_, urdf_string_);
      last_time_ = now;
      started_ = true;

      RCLCPP_INFO(get_logger(), "Circle primitive started");
      return; 
    }

    double dt = (now - last_time_).seconds();
    last_time_ = now;

    if (!circle_.isDone()) {
      circle_.update(robot_state_, dt);

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,"Circle | progress=%.1f%% | fz=%.2f N",
                           circle_.getProgress() * 100.0,
                           robot_state_.wrench_z_.wrench.force.z);
    }

    geometry_msgs::msg::PoseStamped pose;
    geometry_msgs::msg::WrenchStamped wrench;

    circle_.targetPose(pose);
    circle_.targetWrench(wrench);

    pose_pub_->publish(pose);
    wrench_pub_->publish(wrench);

    if (circle_.isDone()) 
    {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,"Circle primitive finished");
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CircleNode>());
  rclcpp::shutdown();
  return 0;
}