// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/Dataframe.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/dataframe__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Dataframe & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: src_address
  {
    out << "src_address: ";
    rosidl_generator_traits::value_to_yaml(msg.src_address, out);
    out << ", ";
  }

  // member: dst_address
  {
    out << "dst_address: ";
    rosidl_generator_traits::value_to_yaml(msg.dst_address, out);
    out << ", ";
  }

  // member: data
  {
    if (msg.data.size() == 0) {
      out << "data: []";
    } else {
      out << "data: [";
      size_t pending_items = msg.data.size();
      for (auto item : msg.data) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: rssi
  {
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Dataframe & msg,
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

  // member: src_address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "src_address: ";
    rosidl_generator_traits::value_to_yaml(msg.src_address, out);
    out << "\n";
  }

  // member: dst_address
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "dst_address: ";
    rosidl_generator_traits::value_to_yaml(msg.dst_address, out);
    out << "\n";
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: rssi
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rssi: ";
    rosidl_generator_traits::value_to_yaml(msg.rssi, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Dataframe & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::Dataframe & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::Dataframe & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::Dataframe>()
{
  return "ros_gz_interfaces::msg::Dataframe";
}

template<>
inline const char * name<ros_gz_interfaces::msg::Dataframe>()
{
  return "ros_gz_interfaces/msg/Dataframe";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::Dataframe>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::Dataframe>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::Dataframe>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__TRAITS_HPP_
