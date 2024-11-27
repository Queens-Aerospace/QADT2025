// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/GuiCamera.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__STRUCT_HPP_

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
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.hpp"
// Member 'track'
#include "ros_gz_interfaces/msg/detail/track_visual__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__GuiCamera __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__GuiCamera __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct GuiCamera_
{
  using Type = GuiCamera_<ContainerAllocator>;

  explicit GuiCamera_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    pose(_init),
    track(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->view_controller = "";
      this->projection_type = "";
    }
  }

  explicit GuiCamera_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    name(_alloc),
    view_controller(_alloc),
    pose(_alloc, _init),
    track(_alloc, _init),
    projection_type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->view_controller = "";
      this->projection_type = "";
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _view_controller_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _view_controller_type view_controller;
  using _pose_type =
    geometry_msgs::msg::Pose_<ContainerAllocator>;
  _pose_type pose;
  using _track_type =
    ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator>;
  _track_type track;
  using _projection_type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _projection_type_type projection_type;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__name(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->name = _arg;
    return *this;
  }
  Type & set__view_controller(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->view_controller = _arg;
    return *this;
  }
  Type & set__pose(
    const geometry_msgs::msg::Pose_<ContainerAllocator> & _arg)
  {
    this->pose = _arg;
    return *this;
  }
  Type & set__track(
    const ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator> & _arg)
  {
    this->track = _arg;
    return *this;
  }
  Type & set__projection_type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->projection_type = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__GuiCamera
    std::shared_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__GuiCamera
    std::shared_ptr<ros_gz_interfaces::msg::GuiCamera_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const GuiCamera_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->view_controller != other.view_controller) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    if (this->track != other.track) {
      return false;
    }
    if (this->projection_type != other.projection_type) {
      return false;
    }
    return true;
  }
  bool operator!=(const GuiCamera_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct GuiCamera_

// alias to use template instance with default allocator
using GuiCamera =
  ros_gz_interfaces::msg::GuiCamera_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__STRUCT_HPP_
