// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/EntityWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__STRUCT_HPP_

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
// Member 'entity'
#include "ros_gz_interfaces/msg/detail/entity__struct.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__EntityWrench __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__EntityWrench __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EntityWrench_
{
  using Type = EntityWrench_<ContainerAllocator>;

  explicit EntityWrench_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    entity(_init),
    wrench(_init)
  {
    (void)_init;
  }

  explicit EntityWrench_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    entity(_alloc, _init),
    wrench(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _entity_type =
    ros_gz_interfaces::msg::Entity_<ContainerAllocator>;
  _entity_type entity;
  using _wrench_type =
    geometry_msgs::msg::Wrench_<ContainerAllocator>;
  _wrench_type wrench;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__entity(
    const ros_gz_interfaces::msg::Entity_<ContainerAllocator> & _arg)
  {
    this->entity = _arg;
    return *this;
  }
  Type & set__wrench(
    const geometry_msgs::msg::Wrench_<ContainerAllocator> & _arg)
  {
    this->wrench = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__EntityWrench
    std::shared_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__EntityWrench
    std::shared_ptr<ros_gz_interfaces::msg::EntityWrench_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EntityWrench_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->entity != other.entity) {
      return false;
    }
    if (this->wrench != other.wrench) {
      return false;
    }
    return true;
  }
  bool operator!=(const EntityWrench_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EntityWrench_

// alias to use template instance with default allocator
using EntityWrench =
  ros_gz_interfaces::msg::EntityWrench_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__STRUCT_HPP_
