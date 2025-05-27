// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from s7_robot_network_interface:msg/RobotStatus.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
#define S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


// Include directives for member types
// Member 'timestamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__s7_robot_network_interface__msg__RobotStatus __attribute__((deprecated))
#else
# define DEPRECATED__s7_robot_network_interface__msg__RobotStatus __declspec(deprecated)
#endif

namespace s7_robot_network_interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RobotStatus_
{
  using Type = RobotStatus_<ContainerAllocator>;

  explicit RobotStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_init),
    pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0ll;
      this->task_stage = 0ll;
      this->align_yaw = false;
    }
  }

  explicit RobotStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : timestamp(_alloc, _init),
    pose(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0ll;
      this->task_stage = 0ll;
      this->align_yaw = false;
    }
  }

  // field types and members
  using _timestamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _timestamp_type timestamp;
  using _robot_id_type =
    int64_t;
  _robot_id_type robot_id;
  using _task_stage_type =
    int64_t;
  _task_stage_type task_stage;
  using _pose_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _pose_type pose;
  using _align_yaw_type =
    bool;
  _align_yaw_type align_yaw;

  // setters for named parameter idiom
  Type & set__timestamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__robot_id(
    const int64_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }
  Type & set__task_stage(
    const int64_t & _arg)
  {
    this->task_stage = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__align_yaw(
    const bool & _arg)
  {
    this->align_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__s7_robot_network_interface__msg__RobotStatus
    std::shared_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__s7_robot_network_interface__msg__RobotStatus
    std::shared_ptr<s7_robot_network_interface::msg::RobotStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RobotStatus_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->robot_id != other.robot_id) {
      return false;
    }
    if (this->task_stage != other.task_stage) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->align_yaw != other.align_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const RobotStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RobotStatus_

// alias to use template instance with default allocator
using RobotStatus =
  s7_robot_network_interface::msg::RobotStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace s7_robot_network_interface

#endif  // S7_ROBOT_NETWORK_INTERFACE__MSG__DETAIL__ROBOT_STATUS__STRUCT_HPP_
