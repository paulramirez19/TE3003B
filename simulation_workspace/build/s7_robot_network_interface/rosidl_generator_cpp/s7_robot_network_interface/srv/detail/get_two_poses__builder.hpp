// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from s7_robot_network_interface:srv/GetTwoPoses.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__BUILDER_HPP_
#define S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__BUILDER_HPP_

#include "s7_robot_network_interface/srv/detail/get_two_poses__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace s7_robot_network_interface
{

namespace srv
{

namespace builder
{

class Init_GetTwoPoses_Request_robot_id
{
public:
  Init_GetTwoPoses_Request_robot_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::s7_robot_network_interface::srv::GetTwoPoses_Request robot_id(::s7_robot_network_interface::srv::GetTwoPoses_Request::_robot_id_type arg)
  {
    msg_.robot_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::s7_robot_network_interface::srv::GetTwoPoses_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::s7_robot_network_interface::srv::GetTwoPoses_Request>()
{
  return s7_robot_network_interface::srv::builder::Init_GetTwoPoses_Request_robot_id();
}

}  // namespace s7_robot_network_interface


namespace s7_robot_network_interface
{

namespace srv
{

namespace builder
{

class Init_GetTwoPoses_Response_delivery_pose
{
public:
  explicit Init_GetTwoPoses_Response_delivery_pose(::s7_robot_network_interface::srv::GetTwoPoses_Response & msg)
  : msg_(msg)
  {}
  ::s7_robot_network_interface::srv::GetTwoPoses_Response delivery_pose(::s7_robot_network_interface::srv::GetTwoPoses_Response::_delivery_pose_type arg)
  {
    msg_.delivery_pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::s7_robot_network_interface::srv::GetTwoPoses_Response msg_;
};

class Init_GetTwoPoses_Response_pickup_pose
{
public:
  Init_GetTwoPoses_Response_pickup_pose()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GetTwoPoses_Response_delivery_pose pickup_pose(::s7_robot_network_interface::srv::GetTwoPoses_Response::_pickup_pose_type arg)
  {
    msg_.pickup_pose = std::move(arg);
    return Init_GetTwoPoses_Response_delivery_pose(msg_);
  }

private:
  ::s7_robot_network_interface::srv::GetTwoPoses_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::s7_robot_network_interface::srv::GetTwoPoses_Response>()
{
  return s7_robot_network_interface::srv::builder::Init_GetTwoPoses_Response_pickup_pose();
}

}  // namespace s7_robot_network_interface

#endif  // S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__BUILDER_HPP_
