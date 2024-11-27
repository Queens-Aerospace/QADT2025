// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/WorldReset.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__WorldReset __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__WorldReset __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorldReset_
{
  using Type = WorldReset_<ContainerAllocator>;

  explicit WorldReset_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->all = false;
      this->time_only = false;
      this->model_only = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->all = false;
      this->time_only = false;
      this->model_only = false;
    }
  }

  explicit WorldReset_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->all = false;
      this->time_only = false;
      this->model_only = false;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->all = false;
      this->time_only = false;
      this->model_only = false;
    }
  }

  // field types and members
  using _all_type =
    bool;
  _all_type all;
  using _time_only_type =
    bool;
  _time_only_type time_only;
  using _model_only_type =
    bool;
  _model_only_type model_only;

  // setters for named parameter idiom
  Type & set__all(
    const bool & _arg)
  {
    this->all = _arg;
    return *this;
  }
  Type & set__time_only(
    const bool & _arg)
  {
    this->time_only = _arg;
    return *this;
  }
  Type & set__model_only(
    const bool & _arg)
  {
    this->model_only = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::WorldReset_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::WorldReset_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::WorldReset_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::WorldReset_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__WorldReset
    std::shared_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__WorldReset
    std::shared_ptr<ros_gz_interfaces::msg::WorldReset_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorldReset_ & other) const
  {
    if (this->all != other.all) {
      return false;
    }
    if (this->time_only != other.time_only) {
      return false;
    }
    if (this->model_only != other.model_only) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorldReset_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorldReset_

// alias to use template instance with default allocator
using WorldReset =
  ros_gz_interfaces::msg::WorldReset_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__STRUCT_HPP_
