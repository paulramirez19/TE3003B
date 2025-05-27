// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from s7_robot_network_interface:srv/GetTwoPoses.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__FUNCTIONS_H_
#define S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "s7_robot_network_interface/msg/rosidl_generator_c__visibility_control.h"

#include "s7_robot_network_interface/srv/detail/get_two_poses__struct.h"

/// Initialize srv/GetTwoPoses message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * s7_robot_network_interface__srv__GetTwoPoses_Request
 * )) before or use
 * s7_robot_network_interface__srv__GetTwoPoses_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Request__init(s7_robot_network_interface__srv__GetTwoPoses_Request * msg);

/// Finalize srv/GetTwoPoses message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Request__fini(s7_robot_network_interface__srv__GetTwoPoses_Request * msg);

/// Create srv/GetTwoPoses message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * s7_robot_network_interface__srv__GetTwoPoses_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
s7_robot_network_interface__srv__GetTwoPoses_Request *
s7_robot_network_interface__srv__GetTwoPoses_Request__create();

/// Destroy srv/GetTwoPoses message.
/**
 * It calls
 * s7_robot_network_interface__srv__GetTwoPoses_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Request__destroy(s7_robot_network_interface__srv__GetTwoPoses_Request * msg);

/// Check for srv/GetTwoPoses message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Request__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Request * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Request * rhs);

/// Copy a srv/GetTwoPoses message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Request__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Request * input,
  s7_robot_network_interface__srv__GetTwoPoses_Request * output);

/// Initialize array of srv/GetTwoPoses messages.
/**
 * It allocates the memory for the number of elements and calls
 * s7_robot_network_interface__srv__GetTwoPoses_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__init(s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * array, size_t size);

/// Finalize array of srv/GetTwoPoses messages.
/**
 * It calls
 * s7_robot_network_interface__srv__GetTwoPoses_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__fini(s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * array);

/// Create array of srv/GetTwoPoses messages.
/**
 * It allocates the memory for the array and calls
 * s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence *
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__create(size_t size);

/// Destroy array of srv/GetTwoPoses messages.
/**
 * It calls
 * s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__destroy(s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * array);

/// Check for srv/GetTwoPoses message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * rhs);

/// Copy an array of srv/GetTwoPoses messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * input,
  s7_robot_network_interface__srv__GetTwoPoses_Request__Sequence * output);

/// Initialize srv/GetTwoPoses message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * s7_robot_network_interface__srv__GetTwoPoses_Response
 * )) before or use
 * s7_robot_network_interface__srv__GetTwoPoses_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Response__init(s7_robot_network_interface__srv__GetTwoPoses_Response * msg);

/// Finalize srv/GetTwoPoses message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Response__fini(s7_robot_network_interface__srv__GetTwoPoses_Response * msg);

/// Create srv/GetTwoPoses message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * s7_robot_network_interface__srv__GetTwoPoses_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
s7_robot_network_interface__srv__GetTwoPoses_Response *
s7_robot_network_interface__srv__GetTwoPoses_Response__create();

/// Destroy srv/GetTwoPoses message.
/**
 * It calls
 * s7_robot_network_interface__srv__GetTwoPoses_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Response__destroy(s7_robot_network_interface__srv__GetTwoPoses_Response * msg);

/// Check for srv/GetTwoPoses message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Response__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Response * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Response * rhs);

/// Copy a srv/GetTwoPoses message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Response__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Response * input,
  s7_robot_network_interface__srv__GetTwoPoses_Response * output);

/// Initialize array of srv/GetTwoPoses messages.
/**
 * It allocates the memory for the number of elements and calls
 * s7_robot_network_interface__srv__GetTwoPoses_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__init(s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * array, size_t size);

/// Finalize array of srv/GetTwoPoses messages.
/**
 * It calls
 * s7_robot_network_interface__srv__GetTwoPoses_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__fini(s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * array);

/// Create array of srv/GetTwoPoses messages.
/**
 * It allocates the memory for the array and calls
 * s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence *
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__create(size_t size);

/// Destroy array of srv/GetTwoPoses messages.
/**
 * It calls
 * s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
void
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__destroy(s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * array);

/// Check for srv/GetTwoPoses message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__are_equal(const s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * lhs, const s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * rhs);

/// Copy an array of srv/GetTwoPoses messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_s7_robot_network_interface
bool
s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence__copy(
  const s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * input,
  s7_robot_network_interface__srv__GetTwoPoses_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__FUNCTIONS_H_
