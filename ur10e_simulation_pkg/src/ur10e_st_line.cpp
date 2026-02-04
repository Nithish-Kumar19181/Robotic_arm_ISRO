#include<urdf/model.h>
#include<algorithm>
#include<kdl/tree.hpp>
#include"rclcpp/rclcpp.hpp"
#include<kdl/chain.hpp>
#include<kdl/tree.hpp>
#include<kdl_parser/kdl_parser.hpp>
#include<std_msgs/msg/string.hpp>
#include<sensor_msgs/msg/joint_state.hpp>
#include<geometry_msgs/msg/pose_stamped.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

using namespace std::placeholders ;
using namespace std::chrono_literals ;

class StraightLine : public rclcpp::Node
{
  public:
  StraightLine()
  :Node("cartesian_st_line_node")
  {
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states",rclcpp::SensorDataQoS(),
      std::bind(&StraightLine::jointCallback,this,_1) 
    );

    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description",rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&StraightLine::robotDescriptionCallback,this,_1) 
    );

    target_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/cartesian_motion_controller/target_frame",rclcpp::QoS(1).transient_local().reliable()
    ) ;

    timer_ = create_wall_timer(20ms,std::bind(&StraightLine::update,this)) ;


  }

  private:

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    last_joint_state_ = *msg ;
    joint_states_received_ = true ;
  }

  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    robot_description_ = msg->data ;
    robot_description_recieved_ = true ;
  }

  void update()
  {
    if(joint_states_received_ == false || robot_description_recieved_ == false)
    {
      return ;
    }

    if(!kdl_init_)
    {
      initKdl() ;
      computeCatesianPose() ;
      motion_start_time_ = now();
    }

    publishTarget() ;
  }

  void initKdl()
  {
    urdf::Model model ;
    model.initString(robot_description_) ;

    KDL::Tree tree ;
    kdl_parser::treeFromUrdfModel(model, tree);

    tree.getChain("base_link","tool0",kdl_chain_) ;

    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_);

    kdl_joints_.resize(kdl_chain_.getNrOfJoints());

    kdl_init_ = true;
    RCLCPP_INFO(get_logger(), "KDL ready");
  }

  void computeCatesianPose()
  {
    kdl_joints_(0) = last_joint_state_.position[0];
    kdl_joints_(1) = last_joint_state_.position[1];
    kdl_joints_(2) = last_joint_state_.position[2];
    kdl_joints_(3) = last_joint_state_.position[3];
    kdl_joints_(4) = last_joint_state_.position[4];
    kdl_joints_(5) = last_joint_state_.position[5];


    KDL::Frame frame;
    fk_solver_->JntToCart(kdl_joints_, frame);

    start_x_ = frame.p.x();
    start_y_ = frame.p.y();
    start_z_ = frame.p.z();
    frame.M.GetQuaternion(qx_, qy_, qz_, qw_);
  }

  double smoothStep(double t)
  {
    
    return 3*t*t - 2*t*t*t;
  }

  void publishTarget()
  {
    double t = (now() - motion_start_time_).seconds() / duration_;
    t = std::clamp(t, 0.0, 1.0);
    double s = smoothStep(t);

    geometry_msgs::msg::PoseStamped target_msg ;
    target_msg.header.stamp = now() ;
    target_msg.header.frame_id = "base_link" ;

    target_msg.pose.position.x = start_x_ - s * distance_ ;
    target_msg.pose.position.y = start_y_ ;
    target_msg.pose.position.z = start_z_ ;
    target_msg.pose.orientation.x = qx_ ;
    target_msg.pose.orientation.y = qy_ ;
    target_msg.pose.orientation.z = qz_ ;
    target_msg.pose.orientation.w = qw_ ;

    target_pub_->publish(target_msg) ;

  }

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_ ;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pub_ ;
  rclcpp::TimerBase::SharedPtr timer_ ;
  rclcpp::Time motion_start_time_ ;


  sensor_msgs::msg::JointState last_joint_state_ ;
  std::string robot_description_ ;

  bool joint_states_received_  = false ;
  bool robot_description_recieved_ = false ;
  bool kdl_init_ = false ;

  KDL::Chain kdl_chain_ ;
  KDL::JntArray kdl_joints_ ;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  double start_x_, start_y_, start_z_;
  double qx_, qy_, qz_, qw_;

  const double distance_ = 0.10;  
  const double duration_ = 5.0;   

};

int main(int argc , char **argv)
{
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<StraightLine>()) ;
  rclcpp::shutdown() ;
  return 0 ;
}