// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from px4_msgs:msg/RoverDifferentialSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__STRUCT_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__px4_msgs__msg__RoverDifferentialSetpoint __attribute__((deprecated))
#else
# define DEPRECATED__px4_msgs__msg__RoverDifferentialSetpoint __declspec(deprecated)
#endif

namespace px4_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RoverDifferentialSetpoint_
{
  using Type = RoverDifferentialSetpoint_<ContainerAllocator>;

  explicit RoverDifferentialSetpoint_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->forward_speed_setpoint = 0.0f;
      this->forward_speed_setpoint_normalized = 0.0f;
      this->yaw_rate_setpoint = 0.0f;
      this->speed_diff_setpoint_normalized = 0.0f;
      this->yaw_setpoint = 0.0f;
    }
  }

  explicit RoverDifferentialSetpoint_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->forward_speed_setpoint = 0.0f;
      this->forward_speed_setpoint_normalized = 0.0f;
      this->yaw_rate_setpoint = 0.0f;
      this->speed_diff_setpoint_normalized = 0.0f;
      this->yaw_setpoint = 0.0f;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _forward_speed_setpoint_type =
    float;
  _forward_speed_setpoint_type forward_speed_setpoint;
  using _forward_speed_setpoint_normalized_type =
    float;
  _forward_speed_setpoint_normalized_type forward_speed_setpoint_normalized;
  using _yaw_rate_setpoint_type =
    float;
  _yaw_rate_setpoint_type yaw_rate_setpoint;
  using _speed_diff_setpoint_normalized_type =
    float;
  _speed_diff_setpoint_normalized_type speed_diff_setpoint_normalized;
  using _yaw_setpoint_type =
    float;
  _yaw_setpoint_type yaw_setpoint;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__forward_speed_setpoint(
    const float & _arg)
  {
    this->forward_speed_setpoint = _arg;
    return *this;
  }
  Type & set__forward_speed_setpoint_normalized(
    const float & _arg)
  {
    this->forward_speed_setpoint_normalized = _arg;
    return *this;
  }
  Type & set__yaw_rate_setpoint(
    const float & _arg)
  {
    this->yaw_rate_setpoint = _arg;
    return *this;
  }
  Type & set__speed_diff_setpoint_normalized(
    const float & _arg)
  {
    this->speed_diff_setpoint_normalized = _arg;
    return *this;
  }
  Type & set__yaw_setpoint(
    const float & _arg)
  {
    this->yaw_setpoint = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator> *;
  using ConstRawPtr =
    const px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__px4_msgs__msg__RoverDifferentialSetpoint
    std::shared_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__px4_msgs__msg__RoverDifferentialSetpoint
    std::shared_ptr<px4_msgs::msg::RoverDifferentialSetpoint_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RoverDifferentialSetpoint_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->forward_speed_setpoint != other.forward_speed_setpoint) {
      return false;
    }
    if (this->forward_speed_setpoint_normalized != other.forward_speed_setpoint_normalized) {
      return false;
    }
    if (this->yaw_rate_setpoint != other.yaw_rate_setpoint) {
      return false;
    }
    if (this->speed_diff_setpoint_normalized != other.speed_diff_setpoint_normalized) {
      return false;
    }
    if (this->yaw_setpoint != other.yaw_setpoint) {
      return false;
    }
    return true;
  }
  bool operator!=(const RoverDifferentialSetpoint_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RoverDifferentialSetpoint_

// alias to use template instance with default allocator
using RoverDifferentialSetpoint =
  px4_msgs::msg::RoverDifferentialSetpoint_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__STRUCT_HPP_
