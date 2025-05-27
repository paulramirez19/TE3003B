// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from s7_robot_network_interface:srv/GetTwoPoses.idl
// generated code does not contain a copyright notice
#include "s7_robot_network_interface/srv/detail/get_two_poses__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

bool
s7_robot_network_interface__srv__GetTwoPoses_Request__init(s7_robot_network_interface__srv__GetTwoPoses_Request * msg)
{
  if (!msg) {
    return false;
  }
  // robot_id
  return true;
}

void
s7_robot_network_interface__srv__GetTwoPoses_Request__fini(s7_robot_network_interface__srv__GetTwoPoses_Request * msg)
{
  if (!msg) {
    return;
  }
  // robot_id
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Request__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Request * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // robot_id
  if (lhs->robot_id != rhs->robot_id) {
    return false;
  }
  return true;
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Request__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Request * input,
  s7_robot_network_interface__srv__GetTwoPoses_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // robot_id
  output->robot_id = input->robot_id;
  return true;
}

s7_robot_network_interface__srv__GetTwoPoses_Request *
s7_robot_network_interface__srv__GetTwoPoses_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__srv__GetTwoPoses_Request * msg = (s7_robot_network_interface__srv__GetTwoPoses_Request *)allocator.allocate(sizeof(s7_robot_network_interface__srv__GetTwoPoses_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(s7_robot_network_interface__srv__GetTwoPoses_Request));
  bool success = s7_robot_network_interface__srv__GetTwoPoses_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
s7_robot_network_interface__srv__GetTwoPoses_Request__destroy(s7_robot_network_interface__srv__GetTwoPoses_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    s7_robot_network_interface__srv__GetTwoPoses_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__init(s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__srv__GetTwoPoses_Request * data = NULL;

  if (size) {
    data = (s7_robot_network_interface__srv__GetTwoPoses_Request *)allocator.zero_allocate(size, sizeof(s7_robot_network_interface__srv__GetTwoPoses_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = s7_robot_network_interface__srv__GetTwoPoses_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        s7_robot_network_interface__srv__GetTwoPoses_Request__fini(&data[i - 1]);
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
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__fini(s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * array)
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
      s7_robot_network_interface__srv__GetTwoPoses_Request__fini(&array->data[i]);
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

s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence *
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * array = (s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence *)allocator.allocate(sizeof(s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__destroy(s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!s7_robot_network_interface__srv__GetTwoPoses_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * input,
  s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(s7_robot_network_interface__srv__GetTwoPoses_Request);
    s7_robot_network_interface__srv__GetTwoPoses_Request * data =
      (s7_robot_network_interface__srv__GetTwoPoses_Request *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!s7_robot_network_interface__srv__GetTwoPoses_Request__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          s7_robot_network_interface__srv__GetTwoPoses_Request__fini(&data[i]);
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
    if (!s7_robot_network_interface__srv__GetTwoPoses_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `pickup_pose`
// Member `delivery_pose`
#include "geometry_msgs/msg/detail/pose2_d__functions.h"

bool
s7_robot_network_interface__srv__GetTwoPoses_Response__init(s7_robot_network_interface__srv__GetTwoPoses_Response * msg)
{
  if (!msg) {
    return false;
  }
  // pickup_pose
  if (!geometry_msgs__msg__Pose2D__init(&msg->pickup_pose)) {
    s7_robot_network_interface__srv__GetTwoPoses_Response__fini(msg);
    return false;
  }
  // delivery_pose
  if (!geometry_msgs__msg__Pose2D__init(&msg->delivery_pose)) {
    s7_robot_network_interface__srv__GetTwoPoses_Response__fini(msg);
    return false;
  }
  return true;
}

void
s7_robot_network_interface__srv__GetTwoPoses_Response__fini(s7_robot_network_interface__srv__GetTwoPoses_Response * msg)
{
  if (!msg) {
    return;
  }
  // pickup_pose
  geometry_msgs__msg__Pose2D__fini(&msg->pickup_pose);
  // delivery_pose
  geometry_msgs__msg__Pose2D__fini(&msg->delivery_pose);
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Response__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Response * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pickup_pose
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->pickup_pose), &(rhs->pickup_pose)))
  {
    return false;
  }
  // delivery_pose
  if (!geometry_msgs__msg__Pose2D__are_equal(
      &(lhs->delivery_pose), &(rhs->delivery_pose)))
  {
    return false;
  }
  return true;
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Response__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Response * input,
  s7_robot_network_interface__srv__GetTwoPoses_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // pickup_pose
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->pickup_pose), &(output->pickup_pose)))
  {
    return false;
  }
  // delivery_pose
  if (!geometry_msgs__msg__Pose2D__copy(
      &(input->delivery_pose), &(output->delivery_pose)))
  {
    return false;
  }
  return true;
}

s7_robot_network_interface__srv__GetTwoPoses_Response *
s7_robot_network_interface__srv__GetTwoPoses_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__srv__GetTwoPoses_Response * msg = (s7_robot_network_interface__srv__GetTwoPoses_Response *)allocator.allocate(sizeof(s7_robot_network_interface__srv__GetTwoPoses_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(s7_robot_network_interface__srv__GetTwoPoses_Response));
  bool success = s7_robot_network_interface__srv__GetTwoPoses_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
s7_robot_network_interface__srv__GetTwoPoses_Response__destroy(s7_robot_network_interface__srv__GetTwoPoses_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    s7_robot_network_interface__srv__GetTwoPoses_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__init(s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__srv__GetTwoPoses_Response * data = NULL;

  if (size) {
    data = (s7_robot_network_interface__srv__GetTwoPoses_Response *)allocator.zero_allocate(size, sizeof(s7_robot_network_interface__srv__GetTwoPoses_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = s7_robot_network_interface__srv__GetTwoPoses_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        s7_robot_network_interface__srv__GetTwoPoses_Response__fini(&data[i - 1]);
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
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__fini(s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * array)
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
      s7_robot_network_interface__srv__GetTwoPoses_Response__fini(&array->data[i]);
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

s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence *
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * array = (s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence *)allocator.allocate(sizeof(s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__destroy(s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!s7_robot_network_interface__srv__GetTwoPoses_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * input,
  s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(s7_robot_network_interface__srv__GetTwoPoses_Response);
    s7_robot_network_interface__srv__GetTwoPoses_Response * data =
      (s7_robot_network_interface__srv__GetTwoPoses_Response *)realloc(output->data, allocation_size);
    if (!data) {
      return false;
    }
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!s7_robot_network_interface__srv__GetTwoPoses_Response__init(&data[i])) {
        /* free currently allocated and return false */
        for (; i-- > output->capacity; ) {
          s7_robot_network_interface__srv__GetTwoPoses_Response__fini(&data[i]);
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
    if (!s7_robot_network_interface__srv__GetTwoPoses_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
