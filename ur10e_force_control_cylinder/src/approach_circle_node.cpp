#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "ur10e_force_control_base/robot_state.hpp"
#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include "ur10e_force_control_cylinder/primitives/approach_primitive.hpp"
#include "ur10e_force_control_cylinder/primitives/circle_primitive.hpp"
#include "ur10e_force_control_cylinder/robot_state_provider.hpp"

using namespace std::placeholders;
using namespace motion_primitives;

class PrimitiveQueueNode : public rclcpp::Node
{
public:
    PrimitiveQueueNode() : Node("primitive_queue_node")
    {
        wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/cartesian_compliance_controller/ft_sensor_wrench",
            rclcpp::SensorDataQoS(),
            std::bind(&PrimitiveQueueNode::wrenchCallback, this, _1));

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&PrimitiveQueueNode::jointCallback, this, _1));

        robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local(),
            std::bind(&PrimitiveQueueNode::robotDescriptionCallback, this, _1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_compliance_controller/target_frame", 10);

        wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/cartesian_compliance_controller/target_wrench", 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PrimitiveQueueNode::timerCallback, this));

        approach_.setParameters(0.35, 3.0, 20.0, 15.0);

        circle_.setParameters(0.25, 2 * M_PI, -20.0, -1);
        circle_.setForceGains(0.5, 0.001, 2.0, 0.01);

        queue_.push_back(&approach_);
        queue_.push_back(&circle_);

        current_ = queue_.begin();
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    RobotState robot_state_;
    robot_state_provider::RobotStateProvider state_provider_;

    ApproachPrimitive approach_;
    CirclePrimitive circle_;

    std::vector<MotionPrimitiveBase*> queue_;
    std::vector<MotionPrimitiveBase*>::iterator current_;

    std::string urdf_string_;

    bool got_joints_{false};
    bool got_wrench_{false};
    bool got_urdf_  {false};

    rclcpp::Time last_time_;
    bool last_time_set_{false};

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
        state_provider_.initialize(urdf_string_, "base_link", "tool0");
        got_urdf_ = true;
        RCLCPP_INFO(get_logger(), "URDF received – kinematics initialised");
    }

    void timerCallback()
    {
        if (!got_joints_ || !got_wrench_ || !got_urdf_)
            return;

        if (!state_provider_.updateRobotState(robot_state_))
            return;

        rclcpp::Time now_time = this->now();
        double dt = last_time_set_ ? (now_time - last_time_).seconds() : 0.01;
        last_time_     = now_time;
        last_time_set_ = true;

        while (current_ != queue_.end() && (*current_)->isDone())
        {
            RCLCPP_INFO(get_logger(), "Primitive done – advancing queue");
            ++current_;
            if (current_ != queue_.end())
                RCLCPP_INFO(get_logger(), "Starting next primitive");
        }

        if (current_ == queue_.end())
        {
            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
                "All primitives completed.");
        }
        else
        {
            MotionPrimitiveBase* active = *current_;

            if (active->getProgress() == 0.0)
                active->onStart(robot_state_, urdf_string_);

            if (!active->isDone())
                active->update(robot_state_, dt);

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "Active primitive progress: %.1f%%  fz=%.2f N",
                active->getProgress() * 100.0,
                robot_state_.wrench_z_.wrench.force.z);
        }

        // ── Always publish (hold last pose/wrench after queue exhausted) ──
        MotionPrimitiveBase* publisher = (current_ == queue_.end())
        ? queue_.back() : *current_;

        geometry_msgs::msg::PoseStamped   pose;
        geometry_msgs::msg::WrenchStamped wrench;

        publisher->targetPose(pose);
        publisher->targetWrench(wrench);

        pose_pub_->publish(pose);
        wrench_pub_->publish(wrench);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrimitiveQueueNode>());
    rclcpp::shutdown();
    return 0;
}