// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/VideoRecord.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/video_record__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const VideoRecord & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: start
  {
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << ", ";
  }

  // member: stop
  {
    out << "stop: ";
    rosidl_generator_traits::value_to_yaml(msg.stop, out);
    out << ", ";
  }

  // member: format
  {
    out << "format: ";
    rosidl_generator_traits::value_to_yaml(msg.format, out);
    out << ", ";
  }

  // member: save_filename
  {
    out << "save_filename: ";
    rosidl_generator_traits::value_to_yaml(msg.save_filename, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const VideoRecord & msg,
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

  // member: start
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "start: ";
    rosidl_generator_traits::value_to_yaml(msg.start, out);
    out << "\n";
  }

  // member: stop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stop: ";
    rosidl_generator_traits::value_to_yaml(msg.stop, out);
    out << "\n";
  }

  // member: format
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "format: ";
    rosidl_generator_traits::value_to_yaml(msg.format, out);
    out << "\n";
  }

  // member: save_filename
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "save_filename: ";
    rosidl_generator_traits::value_to_yaml(msg.save_filename, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const VideoRecord & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::VideoRecord & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::VideoRecord & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::VideoRecord>()
{
  return "ros_gz_interfaces::msg::VideoRecord";
}

template<>
inline const char * name<ros_gz_interfaces::msg::VideoRecord>()
{
  return "ros_gz_interfaces/msg/VideoRecord";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::VideoRecord>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::VideoRecord>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::VideoRecord>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__TRAITS_HPP_
