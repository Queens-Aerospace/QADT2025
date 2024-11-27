// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__Altimeter __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__Altimeter __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Altimeter_
{
  using Type = Altimeter_<ContainerAllocator>;

  explicit Altimeter_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vertical_position = 0.0;
      this->vertical_velocity = 0.0;
      this->vertical_reference = 0.0;
    }
  }

  explicit Altimeter_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->vertical_position = 0.0;
      this->vertical_velocity = 0.0;
      this->vertical_reference = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _vertical_position_type =
    double;
  _vertical_position_type vertical_position;
  using _vertical_velocity_type =
    double;
  _vertical_velocity_type vertical_velocity;
  using _vertical_reference_type =
    double;
  _vertical_reference_type vertical_reference;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__vertical_position(
    const double & _arg)
  {
    this->vertical_position = _arg;
    return *this;
  }
  Type & set__vertical_velocity(
    const double & _arg)
  {
    this->vertical_velocity = _arg;
    return *this;
  }
  Type & set__vertical_reference(
    const double & _arg)
  {
    this->vertical_reference = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::Altimeter_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::Altimeter_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Altimeter_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Altimeter_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__Altimeter
    std::shared_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__Altimeter
    std::shared_ptr<ros_gz_interfaces::msg::Altimeter_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Altimeter_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->vertical_position != other.vertical_position) {
      return false;
    }
    if (this->vertical_velocity != other.vertical_velocity) {
      return false;
    }
    if (this->vertical_reference != other.vertical_reference) {
      return false;
    }
    return true;
  }
  bool operator!=(const Altimeter_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Altimeter_

// alias to use template instance with default allocator
using Altimeter =
  ros_gz_interfaces::msg::Altimeter_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_HPP_
