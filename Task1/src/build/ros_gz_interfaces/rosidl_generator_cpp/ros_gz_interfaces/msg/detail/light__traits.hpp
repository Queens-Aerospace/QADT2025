// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/light__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"
// Member 'diffuse'
// Member 'specular'
#include "std_msgs/msg/detail/color_rgba__traits.hpp"
// Member 'direction'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Light & msg,
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

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: diffuse
  {
    out << "diffuse: ";
    to_flow_style_yaml(msg.diffuse, out);
    out << ", ";
  }

  // member: specular
  {
    out << "specular: ";
    to_flow_style_yaml(msg.specular, out);
    out << ", ";
  }

  // member: attenuation_constant
  {
    out << "attenuation_constant: ";
    rosidl_generator_traits::value_to_yaml(msg.attenuation_constant, out);
    out << ", ";
  }

  // member: attenuation_linear
  {
    out << "attenuation_linear: ";
    rosidl_generator_traits::value_to_yaml(msg.attenuation_linear, out);
    out << ", ";
  }

  // member: attenuation_quadratic
  {
    out << "attenuation_quadratic: ";
    rosidl_generator_traits::value_to_yaml(msg.attenuation_quadratic, out);
    out << ", ";
  }

  // member: direction
  {
    out << "direction: ";
    to_flow_style_yaml(msg.direction, out);
    out << ", ";
  }

  // member: range
  {
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << ", ";
  }

  // member: cast_shadows
  {
    out << "cast_shadows: ";
    rosidl_generator_traits::value_to_yaml(msg.cast_shadows, out);
    out << ", ";
  }

  // member: spot_inner_angle
  {
    out << "spot_inner_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.spot_inner_angle, out);
    out << ", ";
  }

  // member: spot_outer_angle
  {
    out << "spot_outer_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.spot_outer_angle, out);
    out << ", ";
  }

  // member: spot_falloff
  {
    out << "spot_falloff: ";
    rosidl_generator_traits::value_to_yaml(msg.spot_falloff, out);
    out << ", ";
  }

  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: parent_id
  {
    out << "parent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_id, out);
    out << ", ";
  }

  // member: intensity
  {
    out << "intensity: ";
    rosidl_generator_traits::value_to_yaml(msg.intensity, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Light & msg,
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

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
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

  // member: diffuse
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "diffuse:\n";
    to_block_style_yaml(msg.diffuse, out, indentation + 2);
  }

  // member: specular
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "specular:\n";
    to_block_style_yaml(msg.specular, out, indentation + 2);
  }

  // member: attenuation_constant
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attenuation_constant: ";
    rosidl_generator_traits::value_to_yaml(msg.attenuation_constant, out);
    out << "\n";
  }

  // member: attenuation_linear
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attenuation_linear: ";
    rosidl_generator_traits::value_to_yaml(msg.attenuation_linear, out);
    out << "\n";
  }

  // member: attenuation_quadratic
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "attenuation_quadratic: ";
    rosidl_generator_traits::value_to_yaml(msg.attenuation_quadratic, out);
    out << "\n";
  }

  // member: direction
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "direction:\n";
    to_block_style_yaml(msg.direction, out, indentation + 2);
  }

  // member: range
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "range: ";
    rosidl_generator_traits::value_to_yaml(msg.range, out);
    out << "\n";
  }

  // member: cast_shadows
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cast_shadows: ";
    rosidl_generator_traits::value_to_yaml(msg.cast_shadows, out);
    out << "\n";
  }

  // member: spot_inner_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spot_inner_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.spot_inner_angle, out);
    out << "\n";
  }

  // member: spot_outer_angle
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spot_outer_angle: ";
    rosidl_generator_traits::value_to_yaml(msg.spot_outer_angle, out);
    out << "\n";
  }

  // member: spot_falloff
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "spot_falloff: ";
    rosidl_generator_traits::value_to_yaml(msg.spot_falloff, out);
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

  // member: parent_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "parent_id: ";
    rosidl_generator_traits::value_to_yaml(msg.parent_id, out);
    out << "\n";
  }

  // member: intensity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "intensity: ";
    rosidl_generator_traits::value_to_yaml(msg.intensity, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Light & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::Light & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::Light & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::Light>()
{
  return "ros_gz_interfaces::msg::Light";
}

template<>
inline const char * name<ros_gz_interfaces::msg::Light>()
{
  return "ros_gz_interfaces/msg/Light";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::Light>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::Light>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::Light>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__TRAITS_HPP_
