// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/SensorNoise.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/sensor_noise__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const SensorNoise & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: mean
  {
    out << "mean: ";
    rosidl_generator_traits::value_to_yaml(msg.mean, out);
    out << ", ";
  }

  // member: stddev
  {
    out << "stddev: ";
    rosidl_generator_traits::value_to_yaml(msg.stddev, out);
    out << ", ";
  }

  // member: bias_mean
  {
    out << "bias_mean: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_mean, out);
    out << ", ";
  }

  // member: bias_stddev
  {
    out << "bias_stddev: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_stddev, out);
    out << ", ";
  }

  // member: precision
  {
    out << "precision: ";
    rosidl_generator_traits::value_to_yaml(msg.precision, out);
    out << ", ";
  }

  // member: dynamic_bias_stddev
  {
    out << "dynamic_bias_stddev: ";
    rosidl_generator_traits::value_to_yaml(msg.dynamic_bias_stddev, out);
    out << ", ";
  }

  // member: dynamic_bias_correlation_time
  {
    out << "dynamic_bias_correlation_time: ";
    rosidl_generator_traits::value_to_yaml(msg.dynamic_bias_correlation_time, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const SensorNoise & msg,
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

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: mean
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mean: ";
    rosidl_generator_traits::value_to_yaml(msg.mean, out);
    out << "\n";
  }

  // member: stddev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stddev: ";
    rosidl_generator_traits::value_to_yaml(msg.stddev, out);
    out << "\n";
  }

  // member: bias_mean
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_mean: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_mean, out);
    out << "\n";
  }

  // member: bias_stddev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bias_stddev: ";
    rosidl_generator_traits::value_to_yaml(msg.bias_stddev, out);
    out << "\n";
  }

  // member: precision
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "precision: ";
    rosidl_generator_traits::value_to_yaml(msg.precision, out);
    out << "\n";
  }

  // member: dynamic_bias_stddev
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dynamic_bias_stddev: ";
    rosidl_generator_traits::value_to_yaml(msg.dynamic_bias_stddev, out);
    out << "\n";
  }

  // member: dynamic_bias_correlation_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dynamic_bias_correlation_time: ";
    rosidl_generator_traits::value_to_yaml(msg.dynamic_bias_correlation_time, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const SensorNoise & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::SensorNoise & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::SensorNoise & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::SensorNoise>()
{
  return "ros_gz_interfaces::msg::SensorNoise";
}

template<>
inline const char * name<ros_gz_interfaces::msg::SensorNoise>()
{
  return "ros_gz_interfaces/msg/SensorNoise";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::SensorNoise>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::SensorNoise>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ros_gz_interfaces::msg::SensorNoise>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__TRAITS_HPP_
