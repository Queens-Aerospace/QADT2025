// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/GuiCamera.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/gui_camera__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'track'
#include "ros_gz_interfaces/msg/detail/track_visual__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const GuiCamera & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: view_controller
  {
    out << "view_controller: ";
    rosidl_generator_traits::value_to_yaml(msg.view_controller, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: track
  {
    out << "track: ";
    to_flow_style_yaml(msg.track, out);
    out << ", ";
  }

  // member: projection_type
  {
    out << "projection_type: ";
    rosidl_generator_traits::value_to_yaml(msg.projection_type, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const GuiCamera & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: view_controller
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "view_controller: ";
    rosidl_generator_traits::value_to_yaml(msg.view_controller, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose:\n";
    to_block_style_yaml(msg.pose, out, indentation + 2);
  }

  // member: track
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "track:\n";
    to_block_style_yaml(msg.track, out, indentation + 2);
  }

  // member: projection_type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "projection_type: ";
    rosidl_generator_traits::value_to_yaml(msg.projection_type, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const GuiCamera & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace ros_gz_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use ros_gz_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ros_gz_interfaces::msg::GuiCamera & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::GuiCamera & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::GuiCamera>()
{
  return "ros_gz_interfaces::msg::GuiCamera";
}

template<>
inline const char * name<ros_gz_interfaces::msg::GuiCamera>()
{
  return "ros_gz_interfaces/msg/GuiCamera";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::GuiCamera>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::GuiCamera>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::GuiCamera>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__TRAITS_HPP_
