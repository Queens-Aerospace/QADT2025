// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/EntityWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/entity_wrench__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'entity'
#include "ros_gz_interfaces/msg/detail/entity__traits.hpp"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const EntityWrench & msg,
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

  // member: wrench
  {
    out << "wrench: ";
    to_flow_style_yaml(msg.wrench, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EntityWrench & msg,
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

  // member: wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "wrench:\n";
    to_block_style_yaml(msg.wrench, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EntityWrench & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::EntityWrench & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::EntityWrench & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::EntityWrench>()
{
  return "ros_gz_interfaces::msg::EntityWrench";
}

template<>
inline const char * name<ros_gz_interfaces::msg::EntityWrench>()
{
  return "ros_gz_interfaces/msg/EntityWrench";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::EntityWrench>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Wrench>::value && has_fixed_size<ros_gz_interfaces::msg::Entity>::value && has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::EntityWrench>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Wrench>::value && has_bounded_size<ros_gz_interfaces::msg::Entity>::value && has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ros_gz_interfaces::msg::EntityWrench>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__TRAITS_HPP_
