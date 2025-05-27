// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from s7_robot_network_interface:srv/GetTwoPoses.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__TRAITS_HPP_
#define S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__TRAITS_HPP_

#include "s7_robot_network_interface/srv/detail/get_two_poses__struct.hpp"
#include <rosidl_runtime_cpp/traits.hpp>
#include <stdint.h>
#include <type_traits>

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<s7_robot_network_interface::srv::GetTwoPoses_Request>()
{
  return "s7_robot_network_interface::srv::GetTwoPoses_Request";
}

template<>
inline const char * name<s7_robot_network_interface::srv::GetTwoPoses_Request>()
{
  return "s7_robot_network_interface/srv/GetTwoPoses_Request";
}

template<>
struct has_fixed_size<s7_robot_network_interface::srv::GetTwoPoses_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<s7_robot_network_interface::srv::GetTwoPoses_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<s7_robot_network_interface::srv::GetTwoPoses_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'pickup_pose'
// Member 'delivery_pose'
#include "geometry_msgs/msg/detail/pose2_d__traits.hpp"

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<s7_robot_network_interface::srv::GetTwoPoses_Response>()
{
  return "s7_robot_network_interface::srv::GetTwoPoses_Response";
}

template<>
inline const char * name<s7_robot_network_interface::srv::GetTwoPoses_Response>()
{
  return "s7_robot_network_interface/srv/GetTwoPoses_Response";
}

template<>
struct has_fixed_size<s7_robot_network_interface::srv::GetTwoPoses_Response>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct has_bounded_size<s7_robot_network_interface::srv::GetTwoPoses_Response>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Pose2D>::value> {};

template<>
struct is_message<s7_robot_network_interface::srv::GetTwoPoses_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<s7_robot_network_interface::srv::GetTwoPoses>()
{
  return "s7_robot_network_interface::srv::GetTwoPoses";
}

template<>
inline const char * name<s7_robot_network_interface::srv::GetTwoPoses>()
{
  return "s7_robot_network_interface/srv/GetTwoPoses";
}

template<>
struct has_fixed_size<s7_robot_network_interface::srv::GetTwoPoses>
  : std::integral_constant<
    bool,
    has_fixed_size<s7_robot_network_interface::srv::GetTwoPoses_Request>::value &&
    has_fixed_size<s7_robot_network_interface::srv::GetTwoPoses_Response>::value
  >
{
};

template<>
struct has_bounded_size<s7_robot_network_interface::srv::GetTwoPoses>
  : std::integral_constant<
    bool,
    has_bounded_size<s7_robot_network_interface::srv::GetTwoPoses_Request>::value &&
    has_bounded_size<s7_robot_network_interface::srv::GetTwoPoses_Response>::value
  >
{
};

template<>
struct is_service<s7_robot_network_interface::srv::GetTwoPoses>
  : std::true_type
{
};

template<>
struct is_service_request<s7_robot_network_interface::srv::GetTwoPoses_Request>
  : std::true_type
{
};

template<>
struct is_service_response<s7_robot_network_interface::srv::GetTwoPoses_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__TRAITS_HPP_
