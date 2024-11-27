// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/WorldControl.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/world_control__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'reset'
#include "ros_gz_interfaces/msg/detail/world_reset__traits.hpp"
// Member 'run_to_sim_time'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const WorldControl & msg,
  std::ostream & out)
{
  out << "{";
  // member: pause
  {
    out << "pause: ";
    rosidl_generator_traits::value_to_yaml(msg.pause, out);
    out << ", ";
  }

  // member: step
  {
    out << "step: ";
    rosidl_generator_traits::value_to_yaml(msg.step, out);
    out << ", ";
  }

  // member: multi_step
  {
    out << "multi_step: ";
    rosidl_generator_traits::value_to_yaml(msg.multi_step, out);
    out << ", ";
  }

  // member: reset
  {
    out << "reset: ";
    to_flow_style_yaml(msg.reset, out);
    out << ", ";
  }

  // member: seed
  {
    out << "seed: ";
    rosidl_generator_traits::value_to_yaml(msg.seed, out);
    out << ", ";
  }

  // member: run_to_sim_time
  {
    out << "run_to_sim_time: ";
    to_flow_style_yaml(msg.run_to_sim_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const WorldControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pause
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pause: ";
    rosidl_generator_traits::value_to_yaml(msg.pause, out);
    out << "\n";
  }

  // member: step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "step: ";
    rosidl_generator_traits::value_to_yaml(msg.step, out);
    out << "\n";
  }

  // member: multi_step
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "multi_step: ";
    rosidl_generator_traits::value_to_yaml(msg.multi_step, out);
    out << "\n";
  }

  // member: reset
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "reset:\n";
    to_block_style_yaml(msg.reset, out, indentation + 2);
  }

  // member: seed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "seed: ";
    rosidl_generator_traits::value_to_yaml(msg.seed, out);
    out << "\n";
  }

  // member: run_to_sim_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "run_to_sim_time:\n";
    to_block_style_yaml(msg.run_to_sim_time, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const WorldControl & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::WorldControl & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::WorldControl & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::WorldControl>()
{
  return "ros_gz_interfaces::msg::WorldControl";
}

template<>
inline const char * name<ros_gz_interfaces::msg::WorldControl>()
{
  return "ros_gz_interfaces/msg/WorldControl";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::WorldControl>
  : std::integral_constant<bool, has_fixed_size<builtin_interfaces::msg::Time>::value && has_fixed_size<ros_gz_interfaces::msg::WorldReset>::value> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::WorldControl>
  : std::integral_constant<bool, has_bounded_size<builtin_interfaces::msg::Time>::value && has_bounded_size<ros_gz_interfaces::msg::WorldReset>::value> {};

template<>
struct is_message<ros_gz_interfaces::msg::WorldControl>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__TRAITS_HPP_
