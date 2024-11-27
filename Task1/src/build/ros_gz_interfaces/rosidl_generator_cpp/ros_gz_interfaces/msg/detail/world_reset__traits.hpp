// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/WorldReset.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/world_reset__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorldReset & msg,
  std::ostream & out)
{
  out << "{";
  // member: all
  {
    out << "all: ";
    rosidl_generator_traits::value_to_yaml(msg.all, out);
    out << ", ";
  }

  // member: time_only
  {
    out << "time_only: ";
    rosidl_generator_traits::value_to_yaml(msg.time_only, out);
    out << ", ";
  }

  // member: model_only
  {
    out << "model_only: ";
    rosidl_generator_traits::value_to_yaml(msg.model_only, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WorldReset & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: all
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "all: ";
    rosidl_generator_traits::value_to_yaml(msg.all, out);
    out << "\n";
  }

  // member: time_only
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "time_only: ";
    rosidl_generator_traits::value_to_yaml(msg.time_only, out);
    out << "\n";
  }

  // member: model_only
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "model_only: ";
    rosidl_generator_traits::value_to_yaml(msg.model_only, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorldReset & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::WorldReset & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::WorldReset & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::WorldReset>()
{
  return "ros_gz_interfaces::msg::WorldReset";
}

template<>
inline const char * name<ros_gz_interfaces::msg::WorldReset>()
{
  return "ros_gz_interfaces/msg/WorldReset";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::WorldReset>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::WorldReset>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ros_gz_interfaces::msg::WorldReset>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__TRAITS_HPP_
