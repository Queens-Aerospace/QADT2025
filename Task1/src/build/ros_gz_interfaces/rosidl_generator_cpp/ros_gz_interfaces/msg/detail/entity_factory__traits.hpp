// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/entity_factory__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const EntityFactory & msg,
  std::ostream & out)
{
  out << "{";
  // member: name
  {
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << ", ";
  }

  // member: allow_renaming
  {
    out << "allow_renaming: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_renaming, out);
    out << ", ";
  }

  // member: sdf
  {
    out << "sdf: ";
    rosidl_generator_traits::value_to_yaml(msg.sdf, out);
    out << ", ";
  }

  // member: sdf_filename
  {
    out << "sdf_filename: ";
    rosidl_generator_traits::value_to_yaml(msg.sdf_filename, out);
    out << ", ";
  }

  // member: clone_name
  {
    out << "clone_name: ";
    rosidl_generator_traits::value_to_yaml(msg.clone_name, out);
    out << ", ";
  }

  // member: pose
  {
    out << "pose: ";
    to_flow_style_yaml(msg.pose, out);
    out << ", ";
  }

  // member: relative_to
  {
    out << "relative_to: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_to, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const EntityFactory & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "name: ";
    rosidl_generator_traits::value_to_yaml(msg.name, out);
    out << "\n";
  }

  // member: allow_renaming
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "allow_renaming: ";
    rosidl_generator_traits::value_to_yaml(msg.allow_renaming, out);
    out << "\n";
  }

  // member: sdf
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sdf: ";
    rosidl_generator_traits::value_to_yaml(msg.sdf, out);
    out << "\n";
  }

  // member: sdf_filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sdf_filename: ";
    rosidl_generator_traits::value_to_yaml(msg.sdf_filename, out);
    out << "\n";
  }

  // member: clone_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "clone_name: ";
    rosidl_generator_traits::value_to_yaml(msg.clone_name, out);
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

  // member: relative_to
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "relative_to: ";
    rosidl_generator_traits::value_to_yaml(msg.relative_to, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const EntityFactory & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::EntityFactory & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::EntityFactory & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::EntityFactory>()
{
  return "ros_gz_interfaces::msg::EntityFactory";
}

template<>
inline const char * name<ros_gz_interfaces::msg::EntityFactory>()
{
  return "ros_gz_interfaces/msg/EntityFactory";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::EntityFactory>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::EntityFactory>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::EntityFactory>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__TRAITS_HPP_
