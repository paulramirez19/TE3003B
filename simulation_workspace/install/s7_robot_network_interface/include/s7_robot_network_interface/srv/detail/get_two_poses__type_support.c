// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from s7_robot_network_interface:srv/GetTwoPoses.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "s7_robot_network_interface/srv/detail/get_two_poses__rosidl_typesupport_introspection_c.h"
#include "s7_robot_network_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "s7_robot_network_interface/srv/detail/get_two_poses__functions.h"
#include "s7_robot_network_interface/srv/detail/get_two_poses__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  s7_robot_network_interface__srv__GetTwoPoses_Request__init(message_memory);
}

void GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_fini_function(void * message_memory)
{
  s7_robot_network_interface__srv__GetTwoPoses_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_member_array[1] = {
  {
    "robot_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(s7_robot_network_interface__srv__GetTwoPoses_Request, robot_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_members = {
  "s7_robot_network_interface__srv",  // message namespace
  "GetTwoPoses_Request",  // message name
  1,  // number of fields
  sizeof(s7_robot_network_interface__srv__GetTwoPoses_Request),
  GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_member_array,  // message members
  GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_type_support_handle = {
  0,
  &GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_s7_robot_network_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, s7_robot_network_interface, srv, GetTwoPoses_Request)() {
  if (!GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_type_support_handle.typesupport_identifier) {
    GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetTwoPoses_Request__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "s7_robot_network_interface/srv/detail/get_two_poses__rosidl_typesupport_introspection_c.h"
// already included above
// #include "s7_robot_network_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "s7_robot_network_interface/srv/detail/get_two_poses__functions.h"
// already included above
// #include "s7_robot_network_interface/srv/detail/get_two_poses__struct.h"


// Include directives for member types
// Member `pickup_pose`
// Member `delivery_pose`
#include "geometry_msgs/msg/pose2_d.h"
// Member `pickup_pose`
// Member `delivery_pose`
#include "geometry_msgs/msg/detail/pose2_d__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  s7_robot_network_interface__srv__GetTwoPoses_Response__init(message_memory);
}

void GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_fini_function(void * message_memory)
{
  s7_robot_network_interface__srv__GetTwoPoses_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_member_array[2] = {
  {
    "pickup_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(s7_robot_network_interface__srv__GetTwoPoses_Response, pickup_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "delivery_pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(s7_robot_network_interface__srv__GetTwoPoses_Response, delivery_pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_members = {
  "s7_robot_network_interface__srv",  // message namespace
  "GetTwoPoses_Response",  // message name
  2,  // number of fields
  sizeof(s7_robot_network_interface__srv__GetTwoPoses_Response),
  GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_member_array,  // message members
  GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_type_support_handle = {
  0,
  &GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_s7_robot_network_interface
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, s7_robot_network_interface, srv, GetTwoPoses_Response)() {
  GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose2D)();
  GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose2D)();
  if (!GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_type_support_handle.typesupport_identifier) {
    GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &GetTwoPoses_Response__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "s7_robot_network_interface/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "s7_robot_network_interface/srv/detail/get_two_poses__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_service_members = {
  "s7_robot_network_interface__srv",  // service namespace
  "GetTwoPoses",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_Request_message_type_support_handle,
  NULL  // response message
  // s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_Response_message_type_support_handle
};

static rosidl_service_type_support_t s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_service_type_support_handle = {
  0,
  &s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, s7_robot_network_interface, srv, GetTwoPoses_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, s7_robot_network_interface, srv, GetTwoPoses_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_s7_robot_network_interface
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, s7_robot_network_interface, srv, GetTwoPoses)() {
  if (!s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_service_type_support_handle.typesupport_identifier) {
    s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, s7_robot_network_interface, srv, GetTwoPoses_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, s7_robot_network_interface, srv, GetTwoPoses_Response)()->data;
  }

  return &s7_robot_network_interface__srv__detail__get_two_poses__rosidl_typesupport_introspection_c__GetTwoPoses_service_type_support_handle;
}
