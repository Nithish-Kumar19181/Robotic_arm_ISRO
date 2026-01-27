#include <chrono>
#include <map>
#include <memory>
#include <vector>

#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "ur_msgs/srv/set_force_mode.hpp"

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

using namespace std::chrono_literals;

enum State { APPROACH_Z, SLIDE_X };

class ForceModeWallSeekZ : public rclcpp::Node {
public:
  ForceModeWallSeekZ()
      : Node("force_mode_wall_seek_with_orientation"), fz_filtered_(0.0),
        wrench_received_(false), current_force_z_(0.0), state_(APPROACH_Z),
        urdf_loaded_(false), cylinder_estimated_(false), q_current_(6) {
    RCLCPP_INFO(get_logger(),
                "Force Mode: APPROACH_Z → SLIDE_X with Orientation Control");

    // Wrench subscription
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
        "/force_torque_sensor_broadcaster/wrench", rclcpp::SensorDataQoS(),
        std::bind(&ForceModeWallSeekZ::onWrench, this, std::placeholders::_1));

    // URDF subscription for kinematics
    urdf_sub_ = create_subscription<std_msgs::msg::String>(
        "/robot_description", rclcpp::QoS(1).transient_local().reliable(),
        std::bind(&ForceModeWallSeekZ::onURDF, this, std::placeholders::_1));

    // Joint state subscription for FK
    joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&ForceModeWallSeekZ::onJointState, this,
                  std::placeholders::_1));

    // Service clients
    switch_ctrl_client_ =
        create_client<controller_manager_msgs::srv::SwitchController>(
            "/controller_manager/switch_controller");

    start_force_client_ = create_client<ur_msgs::srv::SetForceMode>(
        "/force_mode_controller/start_force_mode");

    stop_force_client_ = create_client<std_srvs::srv::Trigger>(
        "/force_mode_controller/stop_force_mode");

    dashboard_play_client_ =
        create_client<std_srvs::srv::Trigger>("/dashboard_client/play");

    waitForServices();
    startupSequence();

    // 500 Hz control loop
    timer_ = create_wall_timer(
        2ms, std::bind(&ForceModeWallSeekZ::controlLoop, this));
  }

private:
  void waitForServices() {
    dashboard_play_client_->wait_for_service();
    switch_ctrl_client_->wait_for_service();
    start_force_client_->wait_for_service();
    stop_force_client_->wait_for_service();
  }

  void startupSequence() {
    auto play_req = std::make_shared<std_srvs::srv::Trigger::Request>();
    dashboard_play_client_->async_send_request(play_req);
    rclcpp::sleep_for(500ms);

    auto switch_req = std::make_shared<
        controller_manager_msgs::srv::SwitchController::Request>();
    switch_req->deactivate_controllers = {"joint_trajectory_controller"};
    switch_req->activate_controllers = {"force_mode_controller"};
    switch_req->strictness =
        controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    switch_ctrl_client_->async_send_request(switch_req);

    rclcpp::sleep_for(500ms);

    sendForceCommand_Z(0.0);
  }

  // ========== URDF and Kinematics Setup ==========
  void onURDF(const std_msgs::msg::String::SharedPtr msg) {
    if (urdf_loaded_)
      return;

    urdf::Model model;
    model.initString(msg->data);

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
      RCLCPP_ERROR(get_logger(), "Failed to parse URDF to KDL tree");
      return;
    }

    if (!tree.getChain("base_link", "tool0", chain_)) {
      RCLCPP_ERROR(get_logger(),
                   "Failed to get KDL chain from base_link to tool0");
      return;
    }

    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    urdf_loaded_ = true;

    RCLCPP_INFO(get_logger(), "✓ KDL kinematics initialized");
  }

  void onJointState(const sensor_msgs::msg::JointState::SharedPtr msg) {
    if (!urdf_loaded_)
      return;

    // Map joint names to positions
    std::map<std::string, double> joint_map;
    for (size_t i = 0; i < msg->name.size(); i++)
      joint_map[msg->name[i]] = msg->position[i];

    // Fill in UR joint order
    std::vector<std::string> ur_joints = {
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint"};

    for (size_t i = 0; i < 6; i++) {
      if (joint_map.count(ur_joints[i]))
        q_current_(i) = joint_map[ur_joints[i]];
    }
  }

  // ========== Force Commands ==========

  // Z-only (Approach phase)
  void sendForceCommand_Z(double fz) {
    auto req = std::make_shared<ur_msgs::srv::SetForceMode::Request>();
    geometry_msgs::msg::PoseStamped task_frame;
    task_frame.header.frame_id = "tool0_controller";
    task_frame.pose.orientation.x = 1.0;
    task_frame.pose.orientation.y = 0.0;
    task_frame.pose.orientation.z = 0.0;
    task_frame.pose.orientation.w = 0.0;
    req->task_frame = task_frame;

    geometry_msgs::msg::Wrench wrench;
    wrench.force.x = 0.0;
    wrench.force.y = 0.0;
    wrench.force.z = fz;
    req->wrench = wrench;

    req->selection_vector_x = false;
    req->selection_vector_y = false;
    req->selection_vector_z = true;
    req->selection_vector_rx = false;
    req->selection_vector_ry = false;
    req->selection_vector_rz = false;

    geometry_msgs::msg::Twist speed;
    speed.linear.z = 0.012;
    req->speed_limits = speed;

    req->deviation_limits = {0.005, 0.005, 0.02, 0.1, 0.1, 0.1};
    req->type = ur_msgs::srv::SetForceMode::Request::NO_TRANSFORM;
    req->damping_factor = 0.3;
    req->gain_scaling = 0.4;

    start_force_client_->async_send_request(req);
  }

  // X + Z with Rotational Control (Sliding phase)
  void sendForceCommand_XZ_with_rotation(double fx, double fz, double tx,
                                         double ty) {
    auto req = std::make_shared<ur_msgs::srv::SetForceMode::Request>();

    geometry_msgs::msg::PoseStamped task_frame;
    task_frame.header.frame_id = "tool0_controller";
    task_frame.pose.orientation.x = 1.0;
    task_frame.pose.orientation.y = 0.0;
    task_frame.pose.orientation.z = 0.0;
    task_frame.pose.orientation.w = 0.0;
    req->task_frame = task_frame;

    geometry_msgs::msg::Wrench wrench;
    wrench.force.x = fx;
    wrench.force.y = 0.0;
    wrench.force.z = fz;
    wrench.torque.x = tx; // Orientation control
    wrench.torque.y = ty; // Orientation control
    wrench.torque.z = 0.0;
    req->wrench = wrench;

    req->selection_vector_x = true;
    req->selection_vector_y = false;
    req->selection_vector_z = true;
    req->selection_vector_rx = true; // Enable rotational compliance
    req->selection_vector_ry = true; // Enable rotational compliance
    req->selection_vector_rz = false;

    geometry_msgs::msg::Twist speed;
    speed.linear.x = 0.100; // 100 mm/s (reduced from 400 mm/s for stability)
    speed.linear.z = 0.005;
    speed.angular.x = 0.2; // Rotational speed limit
    speed.angular.y = 0.2; // Rotational speed limit
    req->speed_limits = speed;

    req->deviation_limits = {0.005, 0.005, 0.02, 0.3, 0.3, 0.1};
    req->type = ur_msgs::srv::SetForceMode::Request::NO_TRANSFORM;
    req->gain_scaling = 0.15;
    req->damping_factor = 0.85;

    start_force_client_->async_send_request(req);
  }

  // ========== Sensor Callback ==========
  void onWrench(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    wrench_received_ = true;
    double alpha = 0.08;
    fz_filtered_ = (1.0 - alpha) * fz_filtered_ + alpha * msg->wrench.force.z;
  }

  void controlLoop() {
    if (!wrench_received_)
      return;

    double dt = 0.002;
    double ramp_rate = 20.0;
    double target_fz = -10.0; // desired negative force against wall

    switch (state_) {
    case APPROACH_Z: {
      // Ramp up force
      if (current_force_z_ > target_fz) {
        current_force_z_ -= ramp_rate * dt;
        if (current_force_z_ < target_fz)
          current_force_z_ = target_fz;
      }

      sendForceCommand_Z(current_force_z_);

      if (fz_filtered_ < -9.0) {
        if (!urdf_loaded_) {
          RCLCPP_WARN(get_logger(), "CONTACT DETECTED but URDF not loaded - "
                                    "continuing without orientation control");
          current_force_z_ = target_fz;
          state_ = SLIDE_X;
          break;
        }

        RCLCPP_INFO(get_logger(),
                    "CONTACT DETECTED → Estimating cylinder geometry");

        // Get current TCP pose
        KDL::Frame tcp_frame;
        fk_solver_->JntToCart(q_current_, tcp_frame);
        contact_frame_ = tcp_frame;

        // Estimate cylinder center
        // Assumption: we approached along tool Z-axis (radially outward)
        KDL::Vector approach_dir = tcp_frame.M.UnitZ(); // tool Z-axis direction

        // Center is behind contact point, opposite to approach direction
        // Estimate 0.1m offset (adjust based on your cylinder size)
        cylinder_center_ = tcp_frame.p - approach_dir * 0.1;
        cylinder_center_.z(tcp_frame.p.z()); // keep same Z height

        // Calculate radius
        KDL::Vector radial = tcp_frame.p - cylinder_center_;
        radial.z(0); // project to XY plane
        cylinder_radius_ = radial.Norm();

        cylinder_estimated_ = true;

        RCLCPP_INFO(get_logger(),
                    "Cylinder: center=(%.3f, %.3f, %.3f), radius=%.3fm",
                    cylinder_center_.x(), cylinder_center_.y(),
                    cylinder_center_.z(), cylinder_radius_);

        current_force_z_ = target_fz;
        state_ = SLIDE_X;
      }
    } break;

    case SLIDE_X: {
      // Hybrid Z force regulation
      double e_z = target_fz - fz_filtered_;
      double Kz = 0.3;
      double Fz = target_fz + Kz * e_z;

      // Clamp for safety
      double Fz_max = -5.0;
      double Fz_min = -25.0;
      if (Fz > Fz_max)
        Fz = Fz_max;
      if (Fz < Fz_min)
        Fz = Fz_min;

      double Fx = 50.0; // Tangential slide force

      // Orientation control (if kinematics available)
      double torque_x = 0.0;
      double torque_y = 0.0;

      if (urdf_loaded_ && cylinder_estimated_) {
        // Get current TCP pose
        KDL::Frame tcp_frame;
        fk_solver_->JntToCart(q_current_, tcp_frame);

        // Calculate radial vector from center to TCP
        KDL::Vector radial_vec = tcp_frame.p - cylinder_center_;
        radial_vec.z(0); // project to XY plane

        // Desired orientation: Z-axis points toward cylinder center (inward
        // normal)
        KDL::Vector z_desired = -radial_vec;
        z_desired.Normalize();

        // Build orthogonal frame
        KDL::Vector up(0, 0, 1);
        KDL::Vector x_desired = up * z_desired; // tangent direction
        x_desired.Normalize();

        KDL::Vector y_desired = z_desired * x_desired;
        y_desired.Normalize();

        // Construct desired rotation matrix
        KDL::Rotation R_desired(x_desired.x(), y_desired.x(), z_desired.x(),
                                x_desired.y(), y_desired.y(), z_desired.y(),
                                x_desired.z(), y_desired.z(), z_desired.z());

        // Compute orientation error
        KDL::Rotation R_error = R_desired * tcp_frame.M.Inverse();

        // Convert to rotation vector (axis-angle)
        KDL::Vector rot_axis = R_error.GetRot();

        // Apply proportional control for orientation
        double Kp_rot = 0.5; // rotational stiffness
        torque_x = Kp_rot * rot_axis.x();
        torque_y = Kp_rot * rot_axis.y();

        // Log current angle (optional)
        double theta = std::atan2(radial_vec.y(), radial_vec.x());
        static int log_counter = 0;
        if (++log_counter % 250 == 0) { // Log every 0.5s
          RCLCPP_INFO(get_logger(),
                      "Angle: %.1f°, Radius: %.3fm, Fz: %.1fN, Torques: [%.2f, "
                      "%.2f] Nm",
                      theta * 180.0 / M_PI, radial_vec.Norm(), fz_filtered_,
                      torque_x, torque_y);
        }
      }

      sendForceCommand_XZ_with_rotation(Fx, Fz, torque_x, torque_y);
    } break;
    }
  }

  // ========== Member Variables ==========

  // Subscriptions
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr
      wrench_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;

  // Timer and clients
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_ctrl_client_;
  rclcpp::Client<ur_msgs::srv::SetForceMode>::SharedPtr start_force_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_force_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr dashboard_play_client_;

  // Force control state
  double fz_filtered_;
  bool wrench_received_;
  double current_force_z_;
  State state_;

  // Kinematics
  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::JntArray q_current_;
  bool urdf_loaded_;

  // Cylinder geometry
  KDL::Vector cylinder_center_;
  double cylinder_radius_;
  KDL::Frame contact_frame_;
  bool cylinder_estimated_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceModeWallSeekZ>());
  rclcpp::shutdown();
  return 0;
}
