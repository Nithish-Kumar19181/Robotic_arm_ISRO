#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainiksolverpos_lma.hpp>
#include <vector>
#include <cmath>
#include <memory>

using namespace std::chrono_literals;

class CircleTrajectoryPublisher : public rclcpp::Node
{
public:
    CircleTrajectoryPublisher()
    : Node("circle_trajectory_publisher")
    {
        // Declare and get the robot_description parameter
        this->declare_parameter<std::string>("robot_description", "");

        // Subscribe to /joint_states to get a real-time seed for the IK solver
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&CircleTrajectoryPublisher::jointStateCallback, this, std::placeholders::_1));

        // Publisher for the target joint positions
        point_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectoryPoint>(
            "/joint_trajectory_controller/joint_trajectory", 10);
            
        // Initialize KDL and the IK solver using the robot_description
        initializeKDL();

        // Start the publisher timer only after KDL is successfully initialized
        if (kdl_ready_) {
            timer_ = this->create_wall_timer(20ms, std::bind(&CircleTrajectoryPublisher::publishCircleTarget, this));
            start_time_ = this->now();
        }
    }

private:
    void initializeKDL()
    {
        std::string robot_desc_string;
        this->get_parameter("robot_description", robot_desc_string);
        
        if (robot_desc_string.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get robot_description parameter. Is robot_state_publisher running?");
            return;
        }

        KDL::Tree kdl_tree;
        if (!kdl_parser::treeFromString(robot_desc_string, kdl_tree)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to construct KDL tree from URDF.");
            return;
        }

        // Define the kinematic chain from the base of the robot to its tool tip
        std::string base_link = "base_link";
        std::string tip_link = "tool0";
        if (!kdl_tree.getChain(base_link, tip_link, kdl_chain_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get KDL chain from %s to %s.", base_link.c_str(), tip_link.c_str());
            return;
        }

        // Initialize the IK solver
        ik_solver_ = std::make_shared<KDL::ChainIkSolverPos_LMA>(kdl_chain_);
        current_joint_positions_ = KDL::JntArray(kdl_chain_.getNrOfJoints());
        kdl_ready_ = true;
        RCLCPP_INFO(this->get_logger(), "KDL and IK solver initialized successfully.");
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (!kdl_ready_ || msg->name.size() < kdl_chain_.getNrOfJoints()) return;

        // Update the current joint positions. This is used as a "seed" for the IK solver,
        // making the solutions more stable and consistent.
        for (size_t i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
            current_joint_positions_(i) = msg->position[i];
        }
        has_joint_state_ = true;
    }

    void publishCircleTarget()
    {
        if (!kdl_ready_ || !has_joint_state_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Waiting for KDL and initial joint state...");
            return;
        }

        rclcpp::Duration elapsed = this->now() - start_time_;
        double t = elapsed.seconds();

        // --- Define the Circle in Cartesian Space ---
        double radius = 0.3;          // Radius of the circle
        double height = 0.5;          // Height above base
        double frequency = 0.1;       // Hz - how fast the circle completes
        double duration = 10.0;       // Total time for the motion
        
        double angle = 2.0 * M_PI * frequency * t;

        if (t > duration) {
            RCLCPP_INFO_ONCE(this->get_logger(), "Circle trajectory complete.");
            return; // Stop publishing after the duration
        }
        
        // Create the target KDL frame (position + orientation) for the tool
        KDL::Frame target_frame;
        target_frame.p.x(radius * cos(angle));      // X coordinate of circle
        target_frame.p.y(radius * sin(angle));      // Y coordinate of circle
        target_frame.p.z(height);                   // Fixed height
        target_frame.M = KDL::Rotation::RPY(M_PI, 0, angle + M_PI/2); // Keep tool oriented properly

        // --- Solve Inverse Kinematics ---
        KDL::JntArray result_joint_angles(kdl_chain_.getNrOfJoints());
        int ret = ik_solver_->CartToJnt(current_joint_positions_, target_frame, result_joint_angles);

        if (ret >= 0) {
            // If IK is successful, publish the resulting joint angles
            trajectory_msgs::msg::JointTrajectoryPoint point_msg;
            point_msg.positions.resize(kdl_chain_.getNrOfJoints());
            for (unsigned int i = 0; i < kdl_chain_.getNrOfJoints(); ++i) {
                point_msg.positions[i] = result_joint_angles(i);
            }
            point_msg.time_from_start = rclcpp::Duration::from_seconds(0.1); // Small duration for controller smoothing
            point_pub_->publish(point_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Inverse Kinematics failed for the target pose at time t=%.2f", t);
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectoryPoint>::SharedPtr point_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time start_time_;

    // KDL and IK related members
    bool kdl_ready_ = false;
    bool has_joint_state_ = false;
    KDL::Chain kdl_chain_;
    std::shared_ptr<KDL::ChainIkSolverPos_LMA> ik_solver_;
    KDL::JntArray current_joint_positions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleTrajectoryPublisher>());
    rclcpp::shutdown();
    return 0;
}