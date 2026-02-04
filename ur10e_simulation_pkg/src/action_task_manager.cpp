#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include "robot_task_interfaces/action/execute_skill.hpp"
#include <chrono>
using ExecuteSkill = robot_task_interfaces::action::ExecuteSkill;
using GoalHandleExecuteSkill = rclcpp_action::ClientGoalHandle<ExecuteSkill>;

using namespace std::placeholders ;
using namespace std::chrono_literals ;

class TaskManager : public rclcpp::Node
{
public:
  TaskManager() : Node("task_manager")
  {
    client_a_ = rclcpp_action::create_client<ExecuteSkill>(
      this, "/approach_then_circle_force");

    client_b_ = rclcpp_action::create_client<ExecuteSkill>(
      this, "/move_to_origin_then_helix");

    timer_ = create_wall_timer(
      1s, std::bind(&TaskManager::start, this));
  }

private:
  rclcpp_action::Client<ExecuteSkill>::SharedPtr client_a_;
  rclcpp_action::Client<ExecuteSkill>::SharedPtr client_b_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool run_a_{true};

  void start()
  {
    timer_->cancel();
    sendNext();
  }

  void sendNext()
  {
    auto goal = ExecuteSkill::Goal();
    goal.skill_name = run_a_ ? "approach_circle" : "helix";

    auto client = run_a_ ? client_a_ : client_b_;

    if (!client->wait_for_action_server(2s))
    {
      RCLCPP_ERROR(get_logger(), "Action server not available");
      return;
    }

    rclcpp_action::Client<ExecuteSkill>::SendGoalOptions options;

    options.feedback_callback =
      [this](GoalHandleExecuteSkill::SharedPtr,
             const std::shared_ptr<const ExecuteSkill::Feedback> feedback)
      {
        RCLCPP_INFO(get_logger(),
          "Feedback: %s (%.2f)",
          feedback->state.c_str(),
          feedback->progress);
      };

    options.result_callback =
      [this](const GoalHandleExecuteSkill::WrappedResult & result)
      {
        if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
        {
          RCLCPP_WARN(get_logger(),
            "Skill finished: %s",
            result.result->message.c_str());

          run_a_ = !run_a_;
          sendNext();   // 🔁 LOOP
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Skill failed or canceled");
        }
      };

    client->async_send_goal(goal, options);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TaskManager>());
  rclcpp::shutdown();
  return 0;
}
