// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/TrackVisual.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_HPP_

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
// Member 'xyz'
#include "geometry_msgs/msg/detail/vector3__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__TrackVisual __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__TrackVisual __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrackVisual_
{
  using Type = TrackVisual_<ContainerAllocator>;

  explicit TrackVisual_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    xyz(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->id = 0ul;
      this->inherit_orientation = false;
      this->min_dist = 0.0;
      this->max_dist = 0.0;
      this->is_static = false;
      this->use_model_frame = false;
      this->inherit_yaw = false;
    }
  }

  explicit TrackVisual_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    name(_alloc),
    xyz(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->name = "";
      this->id = 0ul;
      this->inherit_orientation = false;
      this->min_dist = 0.0;
      this->max_dist = 0.0;
      this->is_static = false;
      this->use_model_frame = false;
      this->inherit_yaw = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _name_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _name_type name;
  using _id_type =
    uint32_t;
  _id_type id;
  using _inherit_orientation_type =
    bool;
  _inherit_orientation_type inherit_orientation;
  using _min_dist_type =
    double;
  _min_dist_type min_dist;
  using _max_dist_type =
    double;
  _max_dist_type max_dist;
  using _is_static_type =
    bool;
  _is_static_type is_static;
  using _use_model_frame_type =
    bool;
  _use_model_frame_type use_model_frame;
  using _xyz_type =
    geometry_msgs::msg::Vector3_<ContainerAllocator>;
  _xyz_type xyz;
  using _inherit_yaw_type =
    bool;
  _inherit_yaw_type inherit_yaw;

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
  Type & set__id(
    const uint32_t & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__inherit_orientation(
    const bool & _arg)
  {
    this->inherit_orientation = _arg;
    return *this;
  }
  Type & set__min_dist(
    const double & _arg)
  {
    this->min_dist = _arg;
    return *this;
  }
  Type & set__max_dist(
    const double & _arg)
  {
    this->max_dist = _arg;
    return *this;
  }
  Type & set__is_static(
    const bool & _arg)
  {
    this->is_static = _arg;
    return *this;
  }
  Type & set__use_model_frame(
    const bool & _arg)
  {
    this->use_model_frame = _arg;
    return *this;
  }
  Type & set__xyz(
    const geometry_msgs::msg::Vector3_<ContainerAllocator> & _arg)
  {
    this->xyz = _arg;
    return *this;
  }
  Type & set__inherit_yaw(
    const bool & _arg)
  {
    this->inherit_yaw = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__TrackVisual
    std::shared_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__TrackVisual
    std::shared_ptr<ros_gz_interfaces::msg::TrackVisual_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrackVisual_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->name != other.name) {
      return false;
    }
    if (this->id != other.id) {
      return false;
    }
    if (this->inherit_orientation != other.inherit_orientation) {
      return false;
    }
    if (this->min_dist != other.min_dist) {
      return false;
    }
    if (this->max_dist != other.max_dist) {
      return false;
    }
    if (this->is_static != other.is_static) {
      return false;
    }
    if (this->use_model_frame != other.use_model_frame) {
      return false;
    }
    if (this->xyz != other.xyz) {
      return false;
    }
    if (this->inherit_yaw != other.inherit_yaw) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrackVisual_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrackVisual_

// alias to use template instance with default allocator
using TrackVisual =
  ros_gz_interfaces::msg::TrackVisual_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_HPP_
