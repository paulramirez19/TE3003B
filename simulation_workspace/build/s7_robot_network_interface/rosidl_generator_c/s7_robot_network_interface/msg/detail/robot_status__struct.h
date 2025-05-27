// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from s7_robot_network_interface:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
#define S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__struct.h"

// Struct defined in msg/RobotStatus in the package s7_robot_network_interface.
typedef struct s7_robot_network_interface__msg__RobotStatus
{
  builtin_interfaces__msg__Time timestamp;
  int64_t robot_id;
  int64_t task_stage;
  geometry_msgs__msg__Pose2D pose;
  bool align_yaw;
} s7_robot_network_interface__msg__RobotStatus;

// Struct for a sequence of s7_robot_network_interface__msg__RobotStatus.
typedef struct s7_robot_network_interface__msg__RobotStatus__Sequence
{
  s7_robot_network_interface__msg__RobotStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} s7_robot_network_interface__msg__RobotStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__STRUCT_H_
