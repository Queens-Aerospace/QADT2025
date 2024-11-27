// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/VideoRecord.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_HPP_

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
# define DEPRECATED__ros_gz_interfaces__msg__VideoRecord __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__VideoRecord __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct VideoRecord_
{
  using Type = VideoRecord_<ContainerAllocator>;

  explicit VideoRecord_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
      this->stop = false;
      this->format = "";
      this->save_filename = "";
    }
  }

  explicit VideoRecord_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    format(_alloc),
    save_filename(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->start = false;
      this->stop = false;
      this->format = "";
      this->save_filename = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _start_type =
    bool;
  _start_type start;
  using _stop_type =
    bool;
  _stop_type stop;
  using _format_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _format_type format;
  using _save_filename_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _save_filename_type save_filename;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__start(
    const bool & _arg)
  {
    this->start = _arg;
    return *this;
  }
  Type & set__stop(
    const bool & _arg)
  {
    this->stop = _arg;
    return *this;
  }
  Type & set__format(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->format = _arg;
    return *this;
  }
  Type & set__save_filename(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->save_filename = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__VideoRecord
    std::shared_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__VideoRecord
    std::shared_ptr<ros_gz_interfaces::msg::VideoRecord_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const VideoRecord_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->start != other.start) {
      return false;
    }
    if (this->stop != other.stop) {
      return false;
    }
    if (this->format != other.format) {
      return false;
    }
    if (this->save_filename != other.save_filename) {
      return false;
    }
    return true;
  }
  bool operator!=(const VideoRecord_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct VideoRecord_

// alias to use template instance with default allocator
using VideoRecord =
  ros_gz_interfaces::msg::VideoRecord_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_HPP_
