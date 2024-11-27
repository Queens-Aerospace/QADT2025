// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/Entity.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__Entity __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__Entity __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Entity_
{
  using Type = Entity_<ContainerAllocator>;

  explicit Entity_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->name = "";
      this->type = 0;
    }
  }

  explicit Entity_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : name(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0ull;
      this->name = "";
      this->type = 0;
    }
  }

  // field types and members
  using _id_type =
    uint64_t;
  _id_type id;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _type_type =
    uint8_t;
  _type_type type;

  // setters for named parameter idiom
  Type & set__id(
    const uint64_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NONE =
    0u;
  static constexpr uint8_t LIGHT =
    1u;
  static constexpr uint8_t MODEL =
    2u;
  static constexpr uint8_t LINK =
    3u;
  static constexpr uint8_t VISUAL =
    4u;
  static constexpr uint8_t COLLISION =
    5u;
  static constexpr uint8_t SENSOR =
    6u;
  static constexpr uint8_t JOINT =
    7u;

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::Entity_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::Entity_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Entity_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Entity_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__Entity
    std::shared_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__Entity
    std::shared_ptr<ros_gz_interfaces::msg::Entity_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Entity_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    return true;
  }
  bool operator!=(const Entity_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Entity_

// alias to use template instance with default allocator
using Entity =
  ros_gz_interfaces::msg::Entity_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::LIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::MODEL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::LINK;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::VISUAL;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::COLLISION;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::SENSOR;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t Entity_<ContainerAllocator>::JOINT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__STRUCT_HPP_
