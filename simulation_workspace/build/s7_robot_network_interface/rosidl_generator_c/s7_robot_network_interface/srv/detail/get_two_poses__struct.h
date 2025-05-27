// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from s7_robot_network_interface:srv/GetTwoPoses.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__STRUCT_H_
#define S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/GetTwoPoses in the package s7_robot_network_interface.
typedef struct s7_robot_network_interface__srv__GetTwoPoses_Request
{
  int64_t robot_id;
} s7_robot_network_interface__srv__GetTwoPoses_Request;

// Struct for a sequence of s7_robot_network_interface__srv__GetTwoPoses_Request.
typedef struct s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence
{
  s7_robot_network_interface__srv__GetTwoPoses_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'pickup_pose'
// Member 'delivery_pose'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

// Struct defined in srv/GetTwoPoses in the package s7_robot_network_interface.
typedef struct s7_robot_network_interface__srv__GetTwoPoses_Response
{
  geometry_msgs__msg__Pose2D pickup_pose;
  geometry_msgs__msg__Pose2D delivery_pose;
} s7_robot_network_interface__srv__GetTwoPoses_Response;

// Struct for a sequence of s7_robot_network_interface__srv__GetTwoPoses_Response.
typedef struct s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence
{
  s7_robot_network_interface__srv__GetTwoPoses_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__STRUCT_H_
