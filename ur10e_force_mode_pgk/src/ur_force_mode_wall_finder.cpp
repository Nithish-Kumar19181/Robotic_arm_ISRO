#include <chrono>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_msgs/srv/set_force_mode.hpp"

#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

using namespace std::chrono_literals;

class ForceModeWallSeekZ : public rclcpp::Node
{
public:
  ForceModeWallSeekZ()
  : Node("force_mode_wall_seek_z_500hz"),
    fz_filtered_(0.0),
    wrench_received_(false),
    current_force_z_(0.0)
  {
    RCLCPP_INFO(get_logger(),
      "Starting Force Mode Z-axis Wall Seek WITH FORCE RAMP-IN (500 Hz)");

    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      "/wrench",
      rclcpp::SensorDataQoS(),
      std::bind(&ForceModeWallSeekZ::onWrench, this, std::placeholders::_1)
    );

    switch_ctrl_client_ =
      create_client<controller_manager_msgs::srv::SwitchController>(
        "/controller_manager/switch_controller");

    start_force_client_ =
      create_client<ur_msgs::srv::SetForceMode>(
        "/force_mode_controller/start_force_mode");

    stop_force_client_ =
      create_client<std_srvs::srv::Trigger>(
        "/force_mode_controller/stop_force_mode");

    dashboard_play_client_ =
      create_client<std_srvs::srv::Trigger>(
        "/dashboard_client/play");

    waitForServices();
    startupSequence();

    // 500 Hz control loop
    timer_ = create_wall_timer(
      2ms, 
      std::bind(&ForceModeWallSeekZ::controlLoop, this));
  }

private:

  void waitForServices()
  {
    dashboard_play_client_->wait_for_service();
    switch_ctrl_client_->wait_for_service();
    start_force_client_->wait_for_service();
    stop_force_client_->wait_for_service();
  }

  void startupSequence()
  {
    auto play_req =
      std::make_shared<std_srvs::srv::Trigger::Request>();
    dashboard_play_client_->async_send_request(play_req);
    rclcpp::sleep_for(500ms);

    auto switch_req =
      std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();

    switch_req->deactivate_controllers = {
      "scaled_joint_trajectory_controller",
      "forward_position_controller"
    };

    switch_req->activate_controllers = {
      "passthrough_trajectory_controller",
      "force_mode_controller"
    };

    switch_req->strictness =
      controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;

    switch_ctrl_client_->async_send_request(switch_req);
    rclcpp::sleep_for(500ms);

    // Start force mode with ZERO force
    sendForceCommand(0.0);
  }

  void sendForceCommand(double fz)
  {
    auto req =
      std::make_shared<ur_msgs::srv::SetForceMode::Request>();

    geometry_msgs::msg::PoseStamped task_frame;
    task_frame.header.frame_id = "tool0_controller";
    task_frame.pose.position.x = 1.0;
    task_frame.pose.position.y = 0.0;
    task_frame.pose.position.z = 0.0;
    task_frame.pose.orientation.w = 0.0;
    req->task_frame = task_frame;

    geometry_msgs::msg::Wrench wrench;
    wrench.force.x = 0.0;
    wrench.force.y = 0.0;
    wrench.force.z = fz;
    req->wrench = wrench;

    req->selection_vector_x  = false;
    req->selection_vector_y  = false;
    req->selection_vector_z  = true;
    req->selection_vector_rx = false;
    req->selection_vector_ry = false;
    req->selection_vector_rz = false;

    geometry_msgs::msg::Twist speed;
    speed.linear.z = 0.003;  // 3 mm/s
    req->speed_limits = speed;

    req->deviation_limits = {0.005, 0.005, 0.02, 0.1, 0.1, 0.1};

    req->type = ur_msgs::srv::SetForceMode::Request::NO_TRANSFORM;
    req->damping_factor = 0.3;
    req->gain_scaling = 0.4;

    start_force_client_->async_send_request(req);
  }

  void onWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg)
  {
    wrench_received_ = true;

    double alpha = 0.08;   // tuned for 500 Hz
    fz_filtered_ =
      (1.0 - alpha) * fz_filtered_ +
      alpha * msg->wrench.force.z;
  }

  void controlLoop()
  {
    // ---------- FORCE RAMP-IN ----------
    double dt = 0.002;          // 500 Hz
    double ramp_rate = 20.0;    // N/s  (reaches 5N in 0.25s)

    // target probing force
    double target_force_z = -5.0;

    if (current_force_z_ > target_force_z)
    {
      current_force_z_ -= ramp_rate * dt;
      if (current_force_z_ < target_force_z)
        current_force_z_ = target_force_z;
    }

    // Send updated force command
    sendForceCommand(current_force_z_);

    // ---------- CONTACT MONITORING ----------
    if (!wrench_received_) return;

    if (fz_filtered_ < -30.0)
    {
      RCLCPP_WARN(get_logger(),
        "CONTACT DETECTED (Fz < -30 N) → stopping force mode");

      auto stop_req =
        std::make_shared<std_srvs::srv::Trigger::Request>();
      stop_force_client_->async_send_request(stop_req);

      timer_->cancel();
      rclcpp::shutdown();
    }
  }

  // ----------------------------

  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
    wrench_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
    switch_ctrl_client_;

  rclcpp::Client<ur_msgs::srv::SetForceMode>::SharedPtr
    start_force_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    stop_force_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr
    dashboard_play_client_;

  double fz_filtered_;
  bool wrench_received_;

  double current_force_z_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceModeWallSeekZ>());
  rclcpp::shutdown();
  return 0;
}
