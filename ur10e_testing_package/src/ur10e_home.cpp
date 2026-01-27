#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

#include <memory>
#include <vector>

class MoveHomeJointNode : public rclcpp::Node
{
public:
  MoveHomeJointNode()
  : Node("move_home_joint_node")
  {
  }

  void init()
  {
    using moveit::planning_interface::MoveGroupInterface;

    move_group_ =
      std::make_shared<MoveGroupInterface>(
        shared_from_this(),
        "ur_manipulator");

    move_group_->setPlanningTime(10.0);
    move_group_->setMaxVelocityScalingFactor(0.1);
    move_group_->setMaxAccelerationScalingFactor(0.1);
    move_group_->setNumPlanningAttempts(1);

    move_to_recorded_home();
  }

private:
  void move_to_recorded_home()
  {
    std::map<std::string, double> home_joints;

    home_joints["shoulder_pan_joint"]  =  0.36214112635006657;
    home_joints["shoulder_lift_joint"] = -1.7531446989678159;
    home_joints["elbow_joint"]         = -2.298771266741955;
    home_joints["wrist_1_joint"]       =  0.9410757643687659;
    home_joints["wrist_2_joint"]       =  1.2120079360659122  ;
    home_joints["wrist_3_joint"]       = -3.1578720449159454;

    move_group_->setJointValueTarget(home_joints);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto result = move_group_->plan(plan);

    if (result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Joint-space planning failed.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Joint-space plan successful. Executing...");

    auto exec_result = move_group_->execute(plan);
    if (exec_result != moveit::core::MoveItErrorCode::SUCCESS)
    {
      RCLCPP_ERROR(get_logger(), "Execution failed.");
      return;
    }

    RCLCPP_INFO(get_logger(), "Moved to Home");
  }


  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MoveHomeJointNode>();
  node->init();

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
