#include <rclcpp/rclcpp.hpp>

#include <lifecycle_msgs/srv/change_state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <unordered_map>
#include <chrono>

using namespace std::chrono_literals;

class ForceTaskSupervisor : public rclcpp::Node
{
public:
  ForceTaskSupervisor()
  : Node("force_task_supervisor")
  {
    RCLCPP_WARN(get_logger(), "Force Task Supervisor started");

    /* ---------- Lifecycle clients ---------- */

    wall_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
      "/wall_detection_node/change_state");

    admittance_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
      "/admittance_control_node/change_state");

    lift_client_ = create_client<lifecycle_msgs::srv::ChangeState>(
      "/cartesian_lift_and_reverse_circle_node/change_state");

    /* ---------- Event subscribers ---------- */

    wall_found_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/wall_found", rclcpp::QoS(1).transient_local(),
      std::bind(&ForceTaskSupervisor::on_wall_found, this, std::placeholders::_1));

    admittance_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/admittance_done", rclcpp::QoS(1).transient_local(),
      std::bind(&ForceTaskSupervisor::on_admittance_done, this, std::placeholders::_1));

    reverse_done_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/reverse_motion_done", rclcpp::QoS(1).transient_local(),
      std::bind(&ForceTaskSupervisor::on_reverse_done, this, std::placeholders::_1));

    /* ---------- FK for height check ---------- */

    urdf_sub_ = create_subscription<std_msgs::msg::String>(
      "/robot_description",
      rclcpp::QoS(1).durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL),
      std::bind(&ForceTaskSupervisor::on_urdf, this, std::placeholders::_1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 50,
      std::bind(&ForceTaskSupervisor::on_joint_state, this, std::placeholders::_1));

    /* ---------- Supervisor timer ---------- */

    timer_ = create_wall_timer(200ms, std::bind(&ForceTaskSupervisor::step, this));
  }

private:

  /* =================== STATE MACHINE =================== */

  enum class State
  {
    START_WALL,
    WAIT_WALL,

    START_ADMITTANCE,
    WAIT_ADMITTANCE,

    START_LIFT,
    WAIT_LIFT,

    CHECK_HEIGHT,
    STOP
  };

  State state_ = State::START_WALL;

  /* =================== FLAGS =================== */

  bool wall_found_ = false;
  bool admittance_done_ = false;
  bool reverse_done_ = false;

  /* =================== LIFECYCLE CLIENTS =================== */

  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr wall_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr admittance_client_;
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr lift_client_;

  /* =================== SUBS =================== */

  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr wall_found_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr admittance_done_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reverse_done_sub_;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr urdf_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  rclcpp::TimerBase::SharedPtr timer_;

  /* =================== FK =================== */

  KDL::Chain chain_;
  std::shared_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  KDL::JntArray q_current_;

  std::unordered_map<std::string, double> joint_map_;
  bool fk_ready_ = false;
  bool joints_ready_ = false;

  double tool_z_ = 0.0;

  /* =================== CALLBACKS =================== */

  void on_wall_found(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      wall_found_ = true;
      RCLCPP_WARN(get_logger(), "Supervisor: wall found event");
    }
  }

  void on_admittance_done(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      admittance_done_ = true;
      RCLCPP_WARN(get_logger(), "Supervisor: admittance done event");
    }
  }

  void on_reverse_done(const std_msgs::msg::Bool::SharedPtr msg)
  {
    if (msg->data)
    {
      reverse_done_ = true;
      RCLCPP_WARN(get_logger(), "Supervisor: reverse motion done event");
    }
  }

  void on_urdf(const std_msgs::msg::String::SharedPtr msg)
  {
    if (fk_solver_) return;

    urdf::Model model;
    if (!model.initString(msg->data)) return;

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) return;

    if (!tree.getChain("base_link", "tool0", chain_)) return;

    fk_solver_ = std::make_shared<KDL::ChainFkSolverPos_recursive>(chain_);
    q_current_ = KDL::JntArray(chain_.getNrOfJoints());

    fk_ready_ = true;

    RCLCPP_INFO(get_logger(), "Supervisor FK ready");
  }

  void on_joint_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    for (size_t i = 0; i < msg->name.size(); ++i)
      joint_map_[msg->name[i]] = msg->position[i];

    joints_ready_ = true;
  }

  /* =================== HELPERS =================== */

  void change_state(
    const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr & client,
    uint8_t transition)
  {
    if (!client->wait_for_service(1s))
    {
      RCLCPP_WARN(get_logger(), "Waiting for lifecycle service...");
      return;
    }

    auto req = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    req->transition.id = transition;
    client->async_send_request(req);
  }

  void activate_node(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr & client)
  {
    change_state(client, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    std::this_thread::sleep_for(200ms);
    change_state(client, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }

  void deactivate_node(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr & client)
  {
    change_state(client, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  double compute_tool_z()
  {
    if (!fk_ready_ || !joints_ready_) return tool_z_;

    unsigned int idx = 0;
    for (const auto &seg : chain_.segments)
    {
      if (seg.getJoint().getType() != KDL::Joint::None)
      {
        const std::string &name = seg.getJoint().getName();
        if (!joint_map_.count(name)) return tool_z_;
        q_current_(idx++) = joint_map_[name];
      }
    }

    KDL::Frame pose;
    fk_solver_->JntToCart(q_current_, pose);

    tool_z_ = pose.p.z();
    return tool_z_;
  }

  /* =================== STATE MACHINE =================== */

  void step()
  {
    switch (state_)
    {
      case State::START_WALL:
        RCLCPP_WARN(get_logger(), "Supervisor: START WALL DETECTION");
        wall_found_ = false;
        activate_node(wall_client_);
        state_ = State::WAIT_WALL;
        break;

      case State::WAIT_WALL:
        if (wall_found_)
        {
          RCLCPP_WARN(get_logger(), "Supervisor: STOP WALL DETECTION");
          deactivate_node(wall_client_);
          state_ = State::START_ADMITTANCE;
        }
        break;

      case State::START_ADMITTANCE:
        RCLCPP_WARN(get_logger(), "Supervisor: START ADMITTANCE");
        admittance_done_ = false;
        activate_node(admittance_client_);
        state_ = State::WAIT_ADMITTANCE;
        break;

      case State::WAIT_ADMITTANCE:
        if (admittance_done_)
        {
          RCLCPP_WARN(get_logger(), "Supervisor: STOP ADMITTANCE");
          deactivate_node(admittance_client_);
          state_ = State::START_LIFT;
        }
        break;

      case State::START_LIFT:
        RCLCPP_WARN(get_logger(), "Supervisor: START LIFT + REVERSE");
        reverse_done_ = false;
        activate_node(lift_client_);
        state_ = State::WAIT_LIFT;
        break;

      case State::WAIT_LIFT:
        if (reverse_done_)
        {
          RCLCPP_WARN(get_logger(), "Supervisor: STOP LIFT NODE");
          deactivate_node(lift_client_);
          state_ = State::CHECK_HEIGHT;
        }
        break;

      case State::CHECK_HEIGHT:
      {
        double z = compute_tool_z();
        RCLCPP_WARN(get_logger(), "Supervisor: TOOL Z = %.3f", z);

        if (z < 1.23)
        {
          RCLCPP_WARN(get_logger(), "Supervisor: RESTART CYCLE");
          state_ = State::START_WALL;
        }
        else
        {
          RCLCPP_ERROR(get_logger(), "Supervisor: PROCESS FINISHED");
          state_ = State::STOP;
        }
        break;
      }

      case State::STOP:
        // Do nothing
        break;
    }
  }
};

/* =================== main =================== */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ForceTaskSupervisor>());
  rclcpp::shutdown();
  return 0;
}
