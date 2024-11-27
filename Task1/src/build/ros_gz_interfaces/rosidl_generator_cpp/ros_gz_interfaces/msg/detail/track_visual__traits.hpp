// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/TrackVisual.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/track_visual__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'xyz'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrackVisual & msg,
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

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: inherit_orientation
  {
    out << "inherit_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.inherit_orientation, out);
    out << ", ";
  }

  // member: min_dist
  {
    out << "min_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.min_dist, out);
    out << ", ";
  }

  // member: max_dist
  {
    out << "max_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.max_dist, out);
    out << ", ";
  }

  // member: is_static
  {
    out << "is_static: ";
    rosidl_generator_traits::value_to_yaml(msg.is_static, out);
    out << ", ";
  }

  // member: use_model_frame
  {
    out << "use_model_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.use_model_frame, out);
    out << ", ";
  }

  // member: xyz
  {
    out << "xyz: ";
    to_flow_style_yaml(msg.xyz, out);
    out << ", ";
  }

  // member: inherit_yaw
  {
    out << "inherit_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.inherit_yaw, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrackVisual & msg,
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

  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: inherit_orientation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "inherit_orientation: ";
    rosidl_generator_traits::value_to_yaml(msg.inherit_orientation, out);
    out << "\n";
  }

  // member: min_dist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "min_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.min_dist, out);
    out << "\n";
  }

  // member: max_dist
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_dist: ";
    rosidl_generator_traits::value_to_yaml(msg.max_dist, out);
    out << "\n";
  }

  // member: is_static
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_static: ";
    rosidl_generator_traits::value_to_yaml(msg.is_static, out);
    out << "\n";
  }

  // member: use_model_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "use_model_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.use_model_frame, out);
    out << "\n";
  }

  // member: xyz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "xyz:\n";
    to_block_style_yaml(msg.xyz, out, indentation + 2);
  }

  // member: inherit_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "inherit_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.inherit_yaw, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrackVisual & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::TrackVisual & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::TrackVisual & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::TrackVisual>()
{
  return "ros_gz_interfaces::msg::TrackVisual";
}

template<>
inline const char * name<ros_gz_interfaces::msg::TrackVisual>()
{
  return "ros_gz_interfaces/msg/TrackVisual";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::TrackVisual>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::TrackVisual>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::TrackVisual>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__TRAITS_HPP_
