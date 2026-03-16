#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include "ur10e_force_control_base/robot_state.hpp"
#include "ur10e_force_control_cylinder/primitives/approach_primitive.hpp"
#include "ur10e_force_control_cylinder/robot_state_provider.hpp"

using namespace std::placeholders;
using namespace motion_primitives;

class ApproachNode : public rclcpp::Node
{
public:

    ApproachNode() : Node("approach_node")
    {

        wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
            "/cartesian_compliance_controller/ft_sensor_wrench",
            rclcpp::SensorDataQoS(),
            std::bind(&ApproachNode::wrenchCallback,this,_1));

        joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            rclcpp::SensorDataQoS(),
            std::bind(&ApproachNode::jointCallback,this,_1));

        robot_desc_sub_ = create_subscription<std_msgs::msg::String>(
            "/robot_description",
            rclcpp::QoS(1).transient_local(),
            std::bind(&ApproachNode::robotDescriptionCallback,this,_1));

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/cartesian_compliance_controller/target_frame",10);

        wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
            "/cartesian_compliance_controller/target_wrench",10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&ApproachNode::timerCallback,this));
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

    std::string urdf_string_;

    bool got_joints_{false};
    bool got_wrench_{false};
    bool got_urdf_{false};

    rclcpp::Time start_time_;

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

        state_provider_.initialize(
            urdf_string_,
            "base_link",
            "tool0");

        got_urdf_ = true;
    }

    void timerCallback()
    {

        if(!got_joints_ || !got_wrench_ || !got_urdf_)
            return;

        if(!state_provider_.updateRobotState(robot_state_))
            return;

        double now = this->get_clock()->now().seconds();

        if(!approach_.isDone())
        {
            if(approach_.getProgress() == 0.0)
            {
                approach_.onStart(robot_state_,urdf_string_);
                start_time_ = this->now();

                RCLCPP_INFO(get_logger(),"Approach started");
            }

            approach_.update(robot_state_,now);
        }

        geometry_msgs::msg::PoseStamped pose;
        geometry_msgs::msg::WrenchStamped wrench;

        approach_.targetPose(pose);
        approach_.targetWrench(wrench);

        pose_pub_->publish(pose);
        wrench_pub_->publish(wrench);

        if(approach_.isDone())
        {
            RCLCPP_INFO_THROTTLE(get_logger(),
                *get_clock(),
                2000,
                "Approach finished");
        }
    }
};

int main(int argc,char **argv)
{
    rclcpp::init(argc,argv);
    rclcpp::spin(std::make_shared<ApproachNode>());
    rclcpp::shutdown();
    return 0;
}