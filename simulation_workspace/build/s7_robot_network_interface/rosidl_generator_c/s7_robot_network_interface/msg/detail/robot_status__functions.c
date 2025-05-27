// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from s7_robot_network_interface:msg/RobotStatus.idl
// generated code does not contain a copyright notice
#include "s7_robot_network_interface/msg/detail/robot_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `timestamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

bool
s7_robot_network_interface__msg__RobotStatus__init(s7_robot_network_interface__msg__RobotStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__init(&msg->timestamp)) {
    s7_robot_network_interface__msg__RobotStatus__fini(msg);
    return false;
  }
  // robot_id
  // task_stage
  // pose
  if (!geometry_msgs__msg__Pose2D__init(&msg->pose)) {
    s7_robot_network_interface__msg__RobotStatus__fini(msg);
    return false;
  }
  // align_yaw
  return true;
}

void
s7_robot_network_interface__msg__RobotStatus__fini(s7_robot_network_interface__msg__RobotStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  builtin_interfaces__msg__Time__fini(&msg->timestamp);
  // robot_id
  // task_stage
  // pose
  geometry_msgs__msg__Pose2D__fini(&msg->pose);
  // align_yaw
}

bool
s7_robot_network_interface__msg__RobotStatus__are_equal(const s7_robot_network_interface__msg__RobotStatus * lhs, const s7_robot_network_interface__msg__RobotStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->timestamp), &(rhs->timestamp)))
  {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  // task_stage
  if (lhs->task_stage != rhs->task_stage) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // align_yaw
  if (lhs->align_yaw != rhs->align_yaw) {
    return false;
  }
  return true;
}

bool
s7_robot_network_interface__msg__RobotStatus__copy(
  const s7_robot_network_interface__msg__RobotStatus * input,
  s7_robot_network_interface__msg__RobotStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->timestamp), &(output->timestamp)))
  {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  // task_stage
  output->task_stage = input->task_stage;
  // pose
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // align_yaw
  output->align_yaw = input->align_yaw;
  return true;
}

s7_robot_network_interface__msg__RobotStatus *
s7_robot_network_interface__msg__RobotStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__msg__RobotStatus * msg = (s7_robot_network_interface__msg__RobotStatus *)allocator.allocate(sizeof(s7_robot_network_interface__msg__RobotStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(s7_robot_network_interface__msg__RobotStatus));
  bool success = s7_robot_network_interface__msg__RobotStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
s7_robot_network_interface__msg__RobotStatus__destroy(s7_robot_network_interface__msg__RobotStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    s7_robot_network_interface__msg__RobotStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
s7_robot_network_interface__msg__RobotStatus__Sequence__init(s7_robot_network_interface__msg__RobotStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__msg__RobotStatus * data = NULL;

  if (size) {
    data = (s7_robot_network_interface__msg__RobotStatus *)allocator.zero_allocate(size, sizeof(s7_robot_network_interface__msg__RobotStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = s7_robot_network_interface__msg__RobotStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        s7_robot_network_interface__msg__RobotStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
s7_robot_network_interface__msg__RobotStatus__Sequence__fini(s7_robot_network_interface__msg__RobotStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      s7_robot_network_interface__msg__RobotStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

s7_robot_network_interface__msg__RobotStatus__Sequence *
s7_robot_network_interface__msg__RobotStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__msg__RobotStatus__Sequence * array = (s7_robot_network_interface__msg__RobotStatus__Sequence *)allocator.allocate(sizeof(s7_robot_network_interface__msg__RobotStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = s7_robot_network_interface__msg__RobotStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
s7_robot_network_interface__msg__RobotStatus__Sequence__destroy(s7_robot_network_interface__msg__RobotStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    s7_robot_network_interface__msg__RobotStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
s7_robot_network_interface__msg__RobotStatus__Sequence__are_equal(const s7_robot_network_interface__msg__RobotStatus__Sequence * lhs, const s7_robot_network_interface__msg__RobotStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!s7_robot_network_interface__msg__RobotStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
s7_robot_network_interface__msg__RobotStatus__Sequence__copy(
  const s7_robot_network_interface__msg__RobotStatus__Sequence * input,
  s7_robot_network_interface__msg__RobotStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(s7_robot_network_interface__msg__RobotStatus);
    s7_robot_network_interface__msg__RobotStatus * data =
      (s7_robot_network_interface__msg__RobotStatus *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!s7_robot_network_interface__msg__RobotStatus__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          s7_robot_network_interface__msg__RobotStatus__fini(&data[i]);
        }
        free(data);
        return false;
      }
    }
    output->data = data;
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!s7_robot_network_interface__msg__RobotStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
