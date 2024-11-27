// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__STRUCT_HPP_

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
// Member 'body_1_name'
// Member 'body_2_name'
#include "std_msgs/msg/detail/string__struct.hpp"
// Member 'body_1_id'
// Member 'body_2_id'
#include "std_msgs/msg/detail/u_int32__struct.hpp"
// Member 'body_1_wrench'
// Member 'body_2_wrench'
#include "geometry_msgs/msg/detail/wrench__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__JointWrench __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__JointWrench __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct JointWrench_
{
  using Type = JointWrench_<ContainerAllocator>;

  explicit JointWrench_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    body_1_name(_init),
    body_1_id(_init),
    body_2_name(_init),
    body_2_id(_init),
    body_1_wrench(_init),
    body_2_wrench(_init)
  {
    (void)_init;
  }

  explicit JointWrench_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    body_1_name(_alloc, _init),
    body_1_id(_alloc, _init),
    body_2_name(_alloc, _init),
    body_2_id(_alloc, _init),
    body_1_wrench(_alloc, _init),
    body_2_wrench(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _body_1_name_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _body_1_name_type body_1_name;
  using _body_1_id_type =
    std_msgs::msg::UInt32_<ContainerAllocator>;
  _body_1_id_type body_1_id;
  using _body_2_name_type =
    std_msgs::msg::String_<ContainerAllocator>;
  _body_2_name_type body_2_name;
  using _body_2_id_type =
    std_msgs::msg::UInt32_<ContainerAllocator>;
  _body_2_id_type body_2_id;
  using _body_1_wrench_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _body_1_wrench_type body_1_wrench;
  using _body_2_wrench_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _body_2_wrench_type body_2_wrench;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__body_1_name(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->body_1_name = _arg;
    return *this;
  }
  Type & set__body_1_id(
    const std_msgs::msg::UInt32_<ContainerAllocator> & _arg)
  {
    this->body_1_id = _arg;
    return *this;
  }
  Type & set__body_2_name(
    const std_msgs::msg::String_<ContainerAllocator> & _arg)
  {
    this->body_2_name = _arg;
    return *this;
  }
  Type & set__body_2_id(
    const std_msgs::msg::UInt32_<ContainerAllocator> & _arg)
  {
    this->body_2_id = _arg;
    return *this;
  }
  Type & set__body_1_wrench(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->body_1_wrench = _arg;
    return *this;
  }
  Type & set__body_2_wrench(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->body_2_wrench = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::JointWrench_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::JointWrench_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__JointWrench
    std::shared_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__JointWrench
    std::shared_ptr<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const JointWrench_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->body_1_name != other.body_1_name) {
      return false;
    }
    if (this->body_1_id != other.body_1_id) {
      return false;
    }
    if (this->body_2_name != other.body_2_name) {
      return false;
    }
    if (this->body_2_id != other.body_2_id) {
      return false;
    }
    if (this->body_1_wrench != other.body_1_wrench) {
      return false;
    }
    if (this->body_2_wrench != other.body_2_wrench) {
      return false;
    }
    return true;
  }
  bool operator!=(const JointWrench_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct JointWrench_

// alias to use template instance with default allocator
using JointWrench =
  ros_gz_interfaces::msg::JointWrench_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__STRUCT_HPP_
