// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/MaterialColor.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/material_color__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'entity'
#include "ros_gz_interfaces/msg/detail/entity__traits.hpp"
// Member 'ambient'
// Member 'diffuse'
// Member 'specular'
// Member 'emissive'
#include "std_msgs/msg/detail/color_rgba__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const MaterialColor & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: entity
  {
    out << "entity: ";
    to_flow_style_yaml(msg.entity, out);
    out << ", ";
  }

  // member: ambient
  {
    out << "ambient: ";
    to_flow_style_yaml(msg.ambient, out);
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

  // member: emissive
  {
    out << "emissive: ";
    to_flow_style_yaml(msg.emissive, out);
    out << ", ";
  }

  // member: shininess
  {
    out << "shininess: ";
    rosidl_generator_traits::value_to_yaml(msg.shininess, out);
    out << ", ";
  }

  // member: entity_match
  {
    out << "entity_match: ";
    rosidl_generator_traits::value_to_yaml(msg.entity_match, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MaterialColor & msg,
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

  // member: entity
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "entity:\n";
    to_block_style_yaml(msg.entity, out, indentation + 2);
  }

  // member: ambient
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "ambient:\n";
    to_block_style_yaml(msg.ambient, out, indentation + 2);
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

  // member: emissive
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "emissive:\n";
    to_block_style_yaml(msg.emissive, out, indentation + 2);
  }

  // member: shininess
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "shininess: ";
    rosidl_generator_traits::value_to_yaml(msg.shininess, out);
    out << "\n";
  }

  // member: entity_match
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "entity_match: ";
    rosidl_generator_traits::value_to_yaml(msg.entity_match, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MaterialColor & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::MaterialColor & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::MaterialColor & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::MaterialColor>()
{
  return "ros_gz_interfaces::msg::MaterialColor";
}

template<>
inline const char * name<ros_gz_interfaces::msg::MaterialColor>()
{
  return "ros_gz_interfaces/msg/MaterialColor";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::MaterialColor>
  : std::integral_constant<bool, has_fixed_size<ros_gz_interfaces::msg::Entity>::value && has_fixed_size<std_msgs::msg::ColorRGBA>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::MaterialColor>
  : std::integral_constant<bool, has_bounded_size<ros_gz_interfaces::msg::Entity>::value && has_bounded_size<std_msgs::msg::ColorRGBA>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ros_gz_interfaces::msg::MaterialColor>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__TRAITS_HPP_
