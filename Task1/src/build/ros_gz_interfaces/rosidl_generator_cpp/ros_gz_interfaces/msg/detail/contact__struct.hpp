// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/Contact.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__CONTACT__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__CONTACT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'collision1'
// Member 'collision2'
#include "ros_gz_interfaces/msg/detail/entity__struct.hpp"
// Member 'positions'
// Member 'normals'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"
// Member 'wrenches'
#include "ros_gz_interfaces/msg/detail/joint_wrench__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__Contact __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__Contact __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Contact_
{
  using Type = Contact_<ContainerAllocator>;

  explicit Contact_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : collision1(_init),
    collision2(_init)
  {
    (void)_init;
  }

  explicit Contact_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : collision1(_alloc, _init),
    collision2(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _collision1_type =
    ros_gz_interfaces::msg::Entity_<ContainerAllocator>;
  _collision1_type collision1;
  using _collision2_type =
    ros_gz_interfaces::msg::Entity_<ContainerAllocator>;
  _collision2_type collision2;
  using _positions_type =
    std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>>;
  _positions_type positions;
  using _normals_type =
    std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>>;
  _normals_type normals;
  using _depths_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _depths_type depths;
  using _wrenches_type =
    std::vector<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>>>;
  _wrenches_type wrenches;

  // setters for named parameter idiom
  Type & set__collision1(
    const ros_gz_interfaces::msg::Entity_<ContainerAllocator> & _arg)
  {
    this->collision1 = _arg;
    return *this;
  }
  Type & set__collision2(
    const ros_gz_interfaces::msg::Entity_<ContainerAllocator> & _arg)
  {
    this->collision2 = _arg;
    return *this;
  }
  Type & set__positions(
    const std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>> & _arg)
  {
    this->positions = _arg;
    return *this;
  }
  Type & set__normals(
    const std::vector<geometry_msgs::msg::Vector3_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<geometry_msgs::msg::Vector3_<ContainerAllocator>>> & _arg)
  {
    this->normals = _arg;
    return *this;
  }
  Type & set__depths(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->depths = _arg;
    return *this;
  }
  Type & set__wrenches(
    const std::vector<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ros_gz_interfaces::msg::JointWrench_<ContainerAllocator>>> & _arg)
  {
    this->wrenches = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::Contact_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::Contact_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Contact_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Contact_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__Contact
    std::shared_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__Contact
    std::shared_ptr<ros_gz_interfaces::msg::Contact_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Contact_ & other) const
  {
    if (this->collision1 != other.collision1) {
      return false;
    }
    if (this->collision2 != other.collision2) {
      return false;
    }
    if (this->positions != other.positions) {
      return false;
    }
    if (this->normals != other.normals) {
      return false;
    }
    if (this->depths != other.depths) {
      return false;
    }
    if (this->wrenches != other.wrenches) {
      return false;
    }
    return true;
  }
  bool operator!=(const Contact_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Contact_

// alias to use template instance with default allocator
using Contact =
  ros_gz_interfaces::msg::Contact_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__CONTACT__STRUCT_HPP_
