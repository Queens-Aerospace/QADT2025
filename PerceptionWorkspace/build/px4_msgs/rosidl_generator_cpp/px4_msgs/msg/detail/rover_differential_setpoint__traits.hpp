// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4_msgs:msg/RoverDifferentialSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__TRAITS_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4_msgs/msg/detail/rover_differential_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RoverDifferentialSetpoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: forward_speed_setpoint
  {
    out << "forward_speed_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_speed_setpoint, out);
    out << ", ";
  }

  // member: forward_speed_setpoint_normalized
  {
    out << "forward_speed_setpoint_normalized: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_speed_setpoint_normalized, out);
    out << ", ";
  }

  // member: yaw_rate_setpoint
  {
    out << "yaw_rate_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate_setpoint, out);
    out << ", ";
  }

  // member: speed_diff_setpoint_normalized
  {
    out << "speed_diff_setpoint_normalized: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_diff_setpoint_normalized, out);
    out << ", ";
  }

  // member: yaw_setpoint
  {
    out << "yaw_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_setpoint, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RoverDifferentialSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: timestamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << "\n";
  }

  // member: forward_speed_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forward_speed_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_speed_setpoint, out);
    out << "\n";
  }

  // member: forward_speed_setpoint_normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "forward_speed_setpoint_normalized: ";
    rosidl_generator_traits::value_to_yaml(msg.forward_speed_setpoint_normalized, out);
    out << "\n";
  }

  // member: yaw_rate_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_rate_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_rate_setpoint, out);
    out << "\n";
  }

  // member: speed_diff_setpoint_normalized
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "speed_diff_setpoint_normalized: ";
    rosidl_generator_traits::value_to_yaml(msg.speed_diff_setpoint_normalized, out);
    out << "\n";
  }

  // member: yaw_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "yaw_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.yaw_setpoint, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RoverDifferentialSetpoint & msg, bool use_flow_style = false)
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

}  // namespace px4_msgs

namespace rosidl_generator_traits
{

[[deprecated("use px4_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4_msgs::msg::RoverDifferentialSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4_msgs::msg::RoverDifferentialSetpoint & msg)
{
  return px4_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4_msgs::msg::RoverDifferentialSetpoint>()
{
  return "px4_msgs::msg::RoverDifferentialSetpoint";
}

template<>
inline const char * name<px4_msgs::msg::RoverDifferentialSetpoint>()
{
  return "px4_msgs/msg/RoverDifferentialSetpoint";
}

template<>
struct has_fixed_size<px4_msgs::msg::RoverDifferentialSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4_msgs::msg::RoverDifferentialSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4_msgs::msg::RoverDifferentialSetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__TRAITS_HPP_
