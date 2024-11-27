// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__EntityFactory __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__EntityFactory __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EntityFactory_
{
  using Type = EntityFactory_<ContainerAllocator>;

  explicit EntityFactory_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pose(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->allow_renaming = false;
      this->relative_to = "world";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->name = "";
      this->allow_renaming = false;
      this->sdf = "";
      this->sdf_filename = "";
      this->clone_name = "";
      this->relative_to = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->sdf = "";
      this->sdf_filename = "";
      this->clone_name = "";
    }
  }

  explicit EntityFactory_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc),
    sdf(_alloc),
    sdf_filename(_alloc),
    clone_name(_alloc),
    pose(_alloc, _init),
    relative_to(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->allow_renaming = false;
      this->relative_to = "world";
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->name = "";
      this->allow_renaming = false;
      this->sdf = "";
      this->sdf_filename = "";
      this->clone_name = "";
      this->relative_to = "";
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->sdf = "";
      this->sdf_filename = "";
      this->clone_name = "";
    }
  }

  // field types and members
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _allow_renaming_type =
    bool;
  _allow_renaming_type allow_renaming;
  using _sdf_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sdf_type sdf;
  using _sdf_filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _sdf_filename_type sdf_filename;
  using _clone_name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _clone_name_type clone_name;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _relative_to_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _relative_to_type relative_to;

  // setters for named parameter idiom
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__allow_renaming(
    const bool & _arg)
  {
    this->allow_renaming = _arg;
    return *this;
  }
  Type & set__sdf(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sdf = _arg;
    return *this;
  }
  Type & set__sdf_filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->sdf_filename = _arg;
    return *this;
  }
  Type & set__clone_name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->clone_name = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__relative_to(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->relative_to = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__EntityFactory
    std::shared_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__EntityFactory
    std::shared_ptr<ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EntityFactory_ & other) const
  {
    if (this->name != other.name) {
      return false;
    }
    if (this->allow_renaming != other.allow_renaming) {
      return false;
    }
    if (this->sdf != other.sdf) {
      return false;
    }
    if (this->sdf_filename != other.sdf_filename) {
      return false;
    }
    if (this->clone_name != other.clone_name) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->relative_to != other.relative_to) {
      return false;
    }
    return true;
  }
  bool operator!=(const EntityFactory_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EntityFactory_

// alias to use template instance with default allocator
using EntityFactory =
  ros_gz_interfaces::msg::EntityFactory_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_HPP_
