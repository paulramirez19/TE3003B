// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from s7_robot_network_interface:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
#define S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_

#include "s7_robot_network_interface/msg/detail/robot_status__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<s7_robot_network_interface::msg::RobotStatus>()
{
  return "s7_robot_network_interface::msg::RobotStatus";
}

template<>
inline const char * name<s7_robot_network_interface::msg::RobotStatus>()
{
  return "s7_robot_network_interface/msg/RobotStatus";
}

template<>
struct has_fixed_size<s7_robot_network_interface::msg::RobotStatus>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct has_bounded_size<s7_robot_network_interface::msg::RobotStatus>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct is_message<s7_robot_network_interface::msg::RobotStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__TRAITS_HPP_
