// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from s7_robot_network_interface:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
#define S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_

#include "s7_robot_network_interface/msg/detail/robot_status__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace s7_robot_network_interface
{

namespace msg
{

namespace builder
{

class Init_RobotStatus_align_yaw
{
public:
  explicit Init_RobotStatus_align_yaw(::s7_robot_network_interface::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  ::s7_robot_network_interface::msg::RobotStatus align_yaw(::s7_robot_network_interface::msg::RobotStatus::_align_yaw_type arg)
  {
    msg_.align_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::s7_robot_network_interface::msg::RobotStatus msg_;
};

class Init_RobotStatus_pose
{
public:
  explicit Init_RobotStatus_pose(::s7_robot_network_interface::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_align_yaw pose(::s7_robot_network_interface::msg::RobotStatus::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_RobotStatus_align_yaw(msg_);
  }

private:
  ::s7_robot_network_interface::msg::RobotStatus msg_;
};

class Init_RobotStatus_task_stage
{
public:
  explicit Init_RobotStatus_task_stage(::s7_robot_network_interface::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_pose task_stage(::s7_robot_network_interface::msg::RobotStatus::_task_stage_type arg)
  {
    msg_.task_stage = std::move(arg);
    return Init_RobotStatus_pose(msg_);
  }

private:
  ::s7_robot_network_interface::msg::RobotStatus msg_;
};

class Init_RobotStatus_robot_id
{
public:
  explicit Init_RobotStatus_robot_id(::s7_robot_network_interface::msg::RobotStatus & msg)
  : msg_(msg)
  {}
  Init_RobotStatus_task_stage robot_id(::s7_robot_network_interface::msg::RobotStatus::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return Init_RobotStatus_task_stage(msg_);
  }

private:
  ::s7_robot_network_interface::msg::RobotStatus msg_;
};

class Init_RobotStatus_timestamp
{
public:
  Init_RobotStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RobotStatus_robot_id timestamp(::s7_robot_network_interface::msg::RobotStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RobotStatus_robot_id(msg_);
  }

private:
  ::s7_robot_network_interface::msg::RobotStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::s7_robot_network_interface::msg::RobotStatus>()
{
  return s7_robot_network_interface::msg::builder::Init_RobotStatus_timestamp();
}

}  // namespace s7_robot_network_interface

#endif  // S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__BUILDER_HPP_
