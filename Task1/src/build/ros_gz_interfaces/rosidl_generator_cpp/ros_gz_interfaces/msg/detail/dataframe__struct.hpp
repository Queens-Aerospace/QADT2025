// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/Dataframe.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_HPP_

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
# define DEPRECATED__ros_gz_interfaces__msg__Dataframe __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__Dataframe __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Dataframe_
{
  using Type = Dataframe_<ContainerAllocator>;

  explicit Dataframe_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->src_address = "";
      this->dst_address = "";
      this->rssi = 0.0;
    }
  }

  explicit Dataframe_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    src_address(_alloc),
    dst_address(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->src_address = "";
      this->dst_address = "";
      this->rssi = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _src_address_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _src_address_type src_address;
  using _dst_address_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _dst_address_type dst_address;
  using _data_type =
    std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>>;
  _data_type data;
  using _rssi_type =
    double;
  _rssi_type rssi;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__src_address(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->src_address = _arg;
    return *this;
  }
  Type & set__dst_address(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->dst_address = _arg;
    return *this;
  }
  Type & set__data(
    const std::vector<uint8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<uint8_t>> & _arg)
  {
    this->data = _arg;
    return *this;
  }
  Type & set__rssi(
    const double & _arg)
  {
    this->rssi = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::Dataframe_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::Dataframe_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Dataframe_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::Dataframe_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__Dataframe
    std::shared_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__Dataframe
    std::shared_ptr<ros_gz_interfaces::msg::Dataframe_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Dataframe_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->src_address != other.src_address) {
      return false;
    }
    if (this->dst_address != other.dst_address) {
      return false;
    }
    if (this->data != other.data) {
      return false;
    }
    if (this->rssi != other.rssi) {
      return false;
    }
    return true;
  }
  bool operator!=(const Dataframe_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Dataframe_

// alias to use template instance with default allocator
using Dataframe =
  ros_gz_interfaces::msg::Dataframe_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_HPP_
