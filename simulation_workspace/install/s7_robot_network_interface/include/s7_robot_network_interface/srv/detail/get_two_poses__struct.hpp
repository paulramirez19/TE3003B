// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from s7_robot_network_interface:srv/GetTwoPoses.idl
// generated code does not contain a copyright notice

#ifndef S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__STRUCT_HPP_
#define S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Request __attribute__((deprecated))
#else
# define DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Request __declspec(deprecated)
#endif

namespace s7_robot_network_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTwoPoses_Request_
{
  using Type = GetTwoPoses_Request_<ContainerAllocator>;

  explicit GetTwoPoses_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0ll;
    }
  }

  explicit GetTwoPoses_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->robot_id = 0ll;
    }
  }

  // field types and members
  using _robot_id_type =
    int64_t;
  _robot_id_type robot_id;

  // setters for named parameter idiom
  Type & set__robot_id(
    const int64_t & _arg)
  {
    this->robot_id = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Request
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Request
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTwoPoses_Request_ & other) const
  {
    if (this->robot_id != other.robot_id) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTwoPoses_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTwoPoses_Request_

// alias to use template instance with default allocator
using GetTwoPoses_Request =
  s7_robot_network_interface::srv::GetTwoPoses_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace s7_robot_network_interface


// Include directives for member types
// Member 'pickup_pose'
// Member 'delivery_pose'
#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Response __attribute__((deprecated))
#else
# define DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Response __declspec(deprecated)
#endif

namespace s7_robot_network_interface
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct GetTwoPoses_Response_
{
  using Type = GetTwoPoses_Response_<ContainerAllocator>;

  explicit GetTwoPoses_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pickup_pose(_init),
    delivery_pose(_init)
  {
    (void)_init;
  }

  explicit GetTwoPoses_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pickup_pose(_alloc, _init),
    delivery_pose(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pickup_pose_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _pickup_pose_type pickup_pose;
  using _delivery_pose_type =
    geometry_msgs::msg::Pose2D_<ContainerAllocator>;
  _delivery_pose_type delivery_pose;

  // setters for named parameter idiom
  Type & set__pickup_pose(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->pickup_pose = _arg;
    return *this;
  }
  Type & set__delivery_pose(
    const geometry_msgs::msg::Pose2D_<ContainerAllocator> & _arg)
  {
    this->delivery_pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Response
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__s7_robot_network_interface__srv__GetTwoPoses_Response
    std::shared_ptr<s7_robot_network_interface::srv::GetTwoPoses_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GetTwoPoses_Response_ & other) const
  {
    if (this->pickup_pose != other.pickup_pose) {
      return false;
    }
    if (this->delivery_pose != other.delivery_pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const GetTwoPoses_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GetTwoPoses_Response_

// alias to use template instance with default allocator
using GetTwoPoses_Response =
  s7_robot_network_interface::srv::GetTwoPoses_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace s7_robot_network_interface

namespace s7_robot_network_interface
{

namespace srv
{

struct GetTwoPoses
{
  using Request = s7_robot_network_interface::srv::GetTwoPoses_Request;
  using Response = s7_robot_network_interface::srv::GetTwoPoses_Response;
};

}  // namespace srv

}  // namespace s7_robot_network_interface

#endif  // S7_ROBOT_NETWORK_INTERFACE__SRV__DETAIL__GET_TWO_POSES__STRUCT_HPP_
