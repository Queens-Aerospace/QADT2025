// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/MaterialColor.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__STRUCT_HPP_

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
// Member 'ambient'
// Member 'diffuse'
// Member 'specular'
// Member 'emissive'
#include "std_msgs/msg/detail/color_rgba__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__MaterialColor __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__MaterialColor __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MaterialColor_
{
  using Type = MaterialColor_<ContainerAllocator>;

  explicit MaterialColor_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    entity(_init),
    ambient(_init),
    diffuse(_init),
    specular(_init),
    emissive(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shininess = 0.0;
      this->entity_match = 0;
    }
  }

  explicit MaterialColor_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    entity(_alloc, _init),
    ambient(_alloc, _init),
    diffuse(_alloc, _init),
    specular(_alloc, _init),
    emissive(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->shininess = 0.0;
      this->entity_match = 0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _entity_type =
    ros_gz_interfaces::msg::Entity_<ContainerAllocator>;
  _entity_type entity;
  using _ambient_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _ambient_type ambient;
  using _diffuse_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _diffuse_type diffuse;
  using _specular_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _specular_type specular;
  using _emissive_type =
    std_msgs::msg::ColorRGBA_<ContainerAllocator>;
  _emissive_type emissive;
  using _shininess_type =
    double;
  _shininess_type shininess;
  using _entity_match_type =
    uint8_t;
  _entity_match_type entity_match;

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
  Type & set__ambient(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->ambient = _arg;
    return *this;
  }
  Type & set__diffuse(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->diffuse = _arg;
    return *this;
  }
  Type & set__specular(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->specular = _arg;
    return *this;
  }
  Type & set__emissive(
    const std_msgs::msg::ColorRGBA_<ContainerAllocator> & _arg)
  {
    this->emissive = _arg;
    return *this;
  }
  Type & set__shininess(
    const double & _arg)
  {
    this->shininess = _arg;
    return *this;
  }
  Type & set__entity_match(
    const uint8_t & _arg)
  {
    this->entity_match = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t FIRST =
    0u;
  static constexpr uint8_t ALL =
    1u;

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__MaterialColor
    std::shared_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__MaterialColor
    std::shared_ptr<ros_gz_interfaces::msg::MaterialColor_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MaterialColor_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->entity != other.entity) {
      return false;
    }
    if (this->ambient != other.ambient) {
      return false;
    }
    if (this->diffuse != other.diffuse) {
      return false;
    }
    if (this->specular != other.specular) {
      return false;
    }
    if (this->emissive != other.emissive) {
      return false;
    }
    if (this->shininess != other.shininess) {
      return false;
    }
    if (this->entity_match != other.entity_match) {
      return false;
    }
    return true;
  }
  bool operator!=(const MaterialColor_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MaterialColor_

// alias to use template instance with default allocator
using MaterialColor =
  ros_gz_interfaces::msg::MaterialColor_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MaterialColor_<ContainerAllocator>::FIRST;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t MaterialColor_<ContainerAllocator>::ALL;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__STRUCT_HPP_
