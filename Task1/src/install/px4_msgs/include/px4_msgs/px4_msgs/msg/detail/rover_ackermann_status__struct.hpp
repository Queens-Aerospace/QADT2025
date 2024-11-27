// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from px4_msgs:msg/RoverAckermannStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__STRUCT_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__px4_msgs__msg__RoverAckermannStatus __attribute__((deprecated))
#else
# define DEPRECATED__px4_msgs__msg__RoverAckermannStatus __declspec(deprecated)
#endif

namespace px4_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RoverAckermannStatus_
{
  using Type = RoverAckermannStatus_<ContainerAllocator>;

  explicit RoverAckermannStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->measured_forward_speed = 0.0f;
      this->adjusted_forward_speed_setpoint = 0.0f;
      this->steering_setpoint_normalized = 0.0f;
      this->adjusted_steering_setpoint_normalized = 0.0f;
      this->measured_lateral_acceleration = 0.0f;
      this->pid_throttle_integral = 0.0f;
      this->pid_lat_accel_integral = 0.0f;
    }
  }

  explicit RoverAckermannStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->timestamp = 0ull;
      this->measured_forward_speed = 0.0f;
      this->adjusted_forward_speed_setpoint = 0.0f;
      this->steering_setpoint_normalized = 0.0f;
      this->adjusted_steering_setpoint_normalized = 0.0f;
      this->measured_lateral_acceleration = 0.0f;
      this->pid_throttle_integral = 0.0f;
      this->pid_lat_accel_integral = 0.0f;
    }
  }

  // field types and members
  using _timestamp_type =
    uint64_t;
  _timestamp_type timestamp;
  using _measured_forward_speed_type =
    float;
  _measured_forward_speed_type measured_forward_speed;
  using _adjusted_forward_speed_setpoint_type =
    float;
  _adjusted_forward_speed_setpoint_type adjusted_forward_speed_setpoint;
  using _steering_setpoint_normalized_type =
    float;
  _steering_setpoint_normalized_type steering_setpoint_normalized;
  using _adjusted_steering_setpoint_normalized_type =
    float;
  _adjusted_steering_setpoint_normalized_type adjusted_steering_setpoint_normalized;
  using _measured_lateral_acceleration_type =
    float;
  _measured_lateral_acceleration_type measured_lateral_acceleration;
  using _pid_throttle_integral_type =
    float;
  _pid_throttle_integral_type pid_throttle_integral;
  using _pid_lat_accel_integral_type =
    float;
  _pid_lat_accel_integral_type pid_lat_accel_integral;

  // setters for named parameter idiom
  Type & set__timestamp(
    const uint64_t & _arg)
  {
    this->timestamp = _arg;
    return *this;
  }
  Type & set__measured_forward_speed(
    const float & _arg)
  {
    this->measured_forward_speed = _arg;
    return *this;
  }
  Type & set__adjusted_forward_speed_setpoint(
    const float & _arg)
  {
    this->adjusted_forward_speed_setpoint = _arg;
    return *this;
  }
  Type & set__steering_setpoint_normalized(
    const float & _arg)
  {
    this->steering_setpoint_normalized = _arg;
    return *this;
  }
  Type & set__adjusted_steering_setpoint_normalized(
    const float & _arg)
  {
    this->adjusted_steering_setpoint_normalized = _arg;
    return *this;
  }
  Type & set__measured_lateral_acceleration(
    const float & _arg)
  {
    this->measured_lateral_acceleration = _arg;
    return *this;
  }
  Type & set__pid_throttle_integral(
    const float & _arg)
  {
    this->pid_throttle_integral = _arg;
    return *this;
  }
  Type & set__pid_lat_accel_integral(
    const float & _arg)
  {
    this->pid_lat_accel_integral = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__px4_msgs__msg__RoverAckermannStatus
    std::shared_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__px4_msgs__msg__RoverAckermannStatus
    std::shared_ptr<px4_msgs::msg::RoverAckermannStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RoverAckermannStatus_ & other) const
  {
    if (this->timestamp != other.timestamp) {
      return false;
    }
    if (this->measured_forward_speed != other.measured_forward_speed) {
      return false;
    }
    if (this->adjusted_forward_speed_setpoint != other.adjusted_forward_speed_setpoint) {
      return false;
    }
    if (this->steering_setpoint_normalized != other.steering_setpoint_normalized) {
      return false;
    }
    if (this->adjusted_steering_setpoint_normalized != other.adjusted_steering_setpoint_normalized) {
      return false;
    }
    if (this->measured_lateral_acceleration != other.measured_lateral_acceleration) {
      return false;
    }
    if (this->pid_throttle_integral != other.pid_throttle_integral) {
      return false;
    }
    if (this->pid_lat_accel_integral != other.pid_lat_accel_integral) {
      return false;
    }
    return true;
  }
  bool operator!=(const RoverAckermannStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RoverAckermannStatus_

// alias to use template instance with default allocator
using RoverAckermannStatus =
  px4_msgs::msg::RoverAckermannStatus_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__STRUCT_HPP_
