#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "ur10e_force_control_base/robot_state.hpp"
#include "ur10e_force_control_base/motion_primitives_base.hpp"
#include "ur10e_force_control_cylinder/primitives/approach_primitive.hpp"
#include "ur10e_force_control_cylinder/primitives/horizontal_primitive.hpp"
#include "ur10e_force_control_cylinder/robot_state_provider.hpp"

using namespace std::placeholders;
using namespace motion_primitives;

/**
 * PrimitiveQueueNode – Approach then slide along a horizontal wall
 *
 * Wall assumption:
 *   • Wall normal: -X
 *   • Robot approaches from +X
 *   • Sliding direction:
 *        +1 → +Y
 *        -1 → -Y
 */
class PrimitiveQueueNode : public rclcpp::Node
{
public:
    PrimitiveQueueNode() : Node("approach_horizontal_node")
    {
        // ── Subscriptions ─────────────────────────────────────────────
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

        // ── Publishers ─────────────────────────────────────────────────
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_compliance_controller/target_frame", 10);

        wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/cartesian_compliance_controller/target_wrench", 10);

        // ── Timer ──────────────────────────────────────────────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&PrimitiveQueueNode::timerCallback, this));

        // ── Primitive parameters ───────────────────────────────────────

        // Approach primitive
        approach_.setParameters(
            0.25,  // speed
            7.0,   // contact threshold
            20.0,  // max force
            15.0   // timeout
        );

        // Horizontal sliding primitive
        horizontal_right_.setParameters(
            0.04,   // linear speed (m/s)
            0.55,    // travel distance (m)
            -10.0,  //  push into wall (-Z)
            +1      // +Y direction
        );

        horizontal_right_.setForceGains(
            1.0,    // kp
            0.001,  // ki
            2.0,    // integral limit
            0.08    // max rate
        );

        // Horizontal sliding primitive
        horizontal_left_.setParameters(
            0.04,   // linear speed (m/s)
            0.55,    // travel distance (m)
            -10.0,  //  push into wall (-Z)
            -1      // -Y direction
        );

        horizontal_left_.setForceGains(
            1.0,    // kp
            0.001,  // ki
            2.0,    // integral limit
            0.06    // max rate
        );

        // ── Queue ─────────────────────────────────────────────────────
        queue_.push_back(&approach_);
        queue_.push_back(&horizontal_right_);
        queue_.push_back(&horizontal_left_);

        current_ = queue_.begin();
    }

private:
    // ── ROS interfaces ────────────────────────────────────────────────
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_desc_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // ── State ─────────────────────────────────────────────────────────
    RobotState robot_state_;
    robot_state_provider::RobotStateProvider state_provider_;

    ApproachPrimitive approach_;
    HorizontalPrimitive horizontal_right_;
    HorizontalPrimitive horizontal_left_;

    std::vector<MotionPrimitiveBase*> queue_;
    std::vector<MotionPrimitiveBase*>::iterator current_;
    MotionPrimitiveBase* last_active_{nullptr};

    std::string urdf_string_;

    bool got_joints_{false};
    bool got_wrench_{false};
    bool got_urdf_{false};

    rclcpp::Time last_time_;
    bool last_time_set_{false};

    // ── Callbacks ─────────────────────────────────────────────────────
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

    // ── Main loop ─────────────────────────────────────────────────────
    void timerCallback()
    {
        if (!got_joints_ || !got_wrench_ || !got_urdf_)
            return;

        if (!state_provider_.updateRobotState(robot_state_))
            return;

        // Compute dt
        rclcpp::Time now_time = this->now();
        double dt = last_time_set_ ? (now_time - last_time_).seconds() : 0.01;
        last_time_ = now_time;
        last_time_set_ = true;

        // Advance queue
        while (current_ != queue_.end() && (*current_)->isDone())
        {
            RCLCPP_INFO(get_logger(), "Primitive '%s' done → next",
                        (*current_)->getName().c_str());
            ++current_;
        }

        if (current_ != queue_.end())
        {
            MotionPrimitiveBase* active = *current_;

            if (active != last_active_)
            {
                last_active_ = active;
                last_time_ = now_time;
                dt = 0.0;
            }

            if (active->getProgress() == 0.0)
                active->onStart(robot_state_, urdf_string_);

            if (!active->isDone())
            {
                if (active == &approach_)
                    active->update(robot_state_, now_time.seconds());
                else
                    active->update(robot_state_, dt);
            }

            RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                "[%s] Progress: %.1f%% | Fz: %.2f N",
                active->getName().c_str(),
                active->getProgress() * 100.0,
                robot_state_.wrench_z_.wrench.force.z);
        }

        // Publish outputs
        MotionPrimitiveBase* publisher =
            (current_ == queue_.end()) ? queue_.back() : *current_;

        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::WrenchStamped wrench;

        publisher->targetPose(pose);
        publisher->targetWrench(wrench);

        pose_pub_->publish(pose);
        wrench_pub_->publish(wrench);
    }
};

// ─────────────────────────────────────────────────────────────────────
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PrimitiveQueueNode>());
    rclcpp::shutdown();
    return 0;
}