// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/ParamVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_HPP_

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
// Member 'params'
#include "rcl_interfaces/msg/detail/parameter__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__ParamVec __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__ParamVec __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ParamVec_
{
  using Type = ParamVec_<ContainerAllocator>;

  explicit ParamVec_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    (void)_init;
  }

  explicit ParamVec_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _params_type =
    std::vector<rcl_interfaces::msg::Parameter_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rcl_interfaces::msg::Parameter_<ContainerAllocator>>>;
  _params_type params;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__params(
    const std::vector<rcl_interfaces::msg::Parameter_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<rcl_interfaces::msg::Parameter_<ContainerAllocator>>> & _arg)
  {
    this->params = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::ParamVec_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::ParamVec_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::ParamVec_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::ParamVec_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__ParamVec
    std::shared_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__ParamVec
    std::shared_ptr<ros_gz_interfaces::msg::ParamVec_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ParamVec_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->params != other.params) {
      return false;
    }
    return true;
  }
  bool operator!=(const ParamVec_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ParamVec_

// alias to use template instance with default allocator
using ParamVec =
  ros_gz_interfaces::msg::ParamVec_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_HPP_
