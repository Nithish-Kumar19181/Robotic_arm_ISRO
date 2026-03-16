#pragma once 

#include <kdl_parser/kdl_parser.hpp>
#include<geometry_msgs/msg/wrench_stamped.hpp>
#include<sensor_msgs/msg/joint_state.hpp>

struct RobotState 
{
    KDL::Frame frame_;
    geometry_msgs::msg::WrenchStamped wrench_z_;
    sensor_msgs::msg::JointState joint_state ;
};
