// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ur_dashboard_msgs:srv/IsInRemoteControl.idl
// generated code does not contain a copyright notice

#ifndef UR_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__BUILDER_HPP_
#define UR_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ur_dashboard_msgs/srv/detail/is_in_remote_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ur_dashboard_msgs
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::srv::IsInRemoteControl_Request>()
{
  return ::ur_dashboard_msgs::srv::IsInRemoteControl_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace ur_dashboard_msgs


namespace ur_dashboard_msgs
{

namespace srv
{

namespace builder
{

class Init_IsInRemoteControl_Response_success
{
public:
  explicit Init_IsInRemoteControl_Response_success(::ur_dashboard_msgs::srv::IsInRemoteControl_Response & msg)
  : msg_(msg)
  {}
  ::ur_dashboard_msgs::srv::IsInRemoteControl_Response success(::ur_dashboard_msgs::srv::IsInRemoteControl_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::IsInRemoteControl_Response msg_;
};

class Init_IsInRemoteControl_Response_remote_control
{
public:
  explicit Init_IsInRemoteControl_Response_remote_control(::ur_dashboard_msgs::srv::IsInRemoteControl_Response & msg)
  : msg_(msg)
  {}
  Init_IsInRemoteControl_Response_success remote_control(::ur_dashboard_msgs::srv::IsInRemoteControl_Response::_remote_control_type arg)
  {
    msg_.remote_control = std::move(arg);
    return Init_IsInRemoteControl_Response_success(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::IsInRemoteControl_Response msg_;
};

class Init_IsInRemoteControl_Response_answer
{
public:
  Init_IsInRemoteControl_Response_answer()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_IsInRemoteControl_Response_remote_control answer(::ur_dashboard_msgs::srv::IsInRemoteControl_Response::_answer_type arg)
  {
    msg_.answer = std::move(arg);
    return Init_IsInRemoteControl_Response_remote_control(msg_);
  }

private:
  ::ur_dashboard_msgs::srv::IsInRemoteControl_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ur_dashboard_msgs::srv::IsInRemoteControl_Response>()
{
  return ur_dashboard_msgs::srv::builder::Init_IsInRemoteControl_Response_answer();
}

}  // namespace ur_dashboard_msgs

#endif  // UR_DASHBOARD_MSGS__SRV__DETAIL__IS_IN_REMOTE_CONTROL__BUILDER_HPP_
