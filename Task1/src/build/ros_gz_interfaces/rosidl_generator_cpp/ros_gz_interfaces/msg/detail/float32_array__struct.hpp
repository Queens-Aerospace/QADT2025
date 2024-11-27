// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/Float32Array.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__Float32Array __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__Float32Array __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Float32Array_
{
  using Type = Float32Array_<ContainerAllocator>;

  explicit Float32Array_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit Float32Array_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _data_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::Float32Array_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::Float32Array_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Float32Array_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Float32Array_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__Float32Array
    std::shared_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__Float32Array
    std::shared_ptr<ros_gz_interfaces::msg::Float32Array_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Float32Array_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const Float32Array_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Float32Array_

// alias to use template instance with default allocator
using Float32Array =
  ros_gz_interfaces::msg::Float32Array_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__STRUCT_HPP_
