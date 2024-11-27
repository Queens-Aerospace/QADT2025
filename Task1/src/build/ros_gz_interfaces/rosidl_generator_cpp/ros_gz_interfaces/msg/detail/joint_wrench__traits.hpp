// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/joint_wrench__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'body_1_name'
// Member 'body_2_name'
#include "std_msgs/msg/detail/string__traits.hpp"
// Member 'body_1_id'
// Member 'body_2_id'
#include "std_msgs/msg/detail/u_int32__traits.hpp"
// Member 'body_1_wrench'
// Member 'body_2_wrench'
#include "geometry_msgs/msg/detail/wrench__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const JointWrench & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: body_1_name
  {
    out << "body_1_name: ";
    to_flow_style_yaml(msg.body_1_name, out);
    out << ", ";
  }

  // member: body_1_id
  {
    out << "body_1_id: ";
    to_flow_style_yaml(msg.body_1_id, out);
    out << ", ";
  }

  // member: body_2_name
  {
    out << "body_2_name: ";
    to_flow_style_yaml(msg.body_2_name, out);
    out << ", ";
  }

  // member: body_2_id
  {
    out << "body_2_id: ";
    to_flow_style_yaml(msg.body_2_id, out);
    out << ", ";
  }

  // member: body_1_wrench
  {
    out << "body_1_wrench: ";
    to_flow_style_yaml(msg.body_1_wrench, out);
    out << ", ";
  }

  // member: body_2_wrench
  {
    out << "body_2_wrench: ";
    to_flow_style_yaml(msg.body_2_wrench, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const JointWrench & msg,
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

  // member: body_1_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_1_name:\n";
    to_block_style_yaml(msg.body_1_name, out, indentation + 2);
  }

  // member: body_1_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_1_id:\n";
    to_block_style_yaml(msg.body_1_id, out, indentation + 2);
  }

  // member: body_2_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_2_name:\n";
    to_block_style_yaml(msg.body_2_name, out, indentation + 2);
  }

  // member: body_2_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_2_id:\n";
    to_block_style_yaml(msg.body_2_id, out, indentation + 2);
  }

  // member: body_1_wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_1_wrench:\n";
    to_block_style_yaml(msg.body_1_wrench, out, indentation + 2);
  }

  // member: body_2_wrench
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "body_2_wrench:\n";
    to_block_style_yaml(msg.body_2_wrench, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const JointWrench & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::JointWrench & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::JointWrench & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::JointWrench>()
{
  return "ros_gz_interfaces::msg::JointWrench";
}

template<>
inline const char * name<ros_gz_interfaces::msg::JointWrench>()
{
  return "ros_gz_interfaces/msg/JointWrench";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::JointWrench>
  : std::integral_constant<bool, has_fixed_size<geometry_msgs::msg::Wrench>::value && has_fixed_size<std_msgs::msg::Header>::value && has_fixed_size<std_msgs::msg::String>::value && has_fixed_size<std_msgs::msg::UInt32>::value> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::JointWrench>
  : std::integral_constant<bool, has_bounded_size<geometry_msgs::msg::Wrench>::value && has_bounded_size<std_msgs::msg::Header>::value && has_bounded_size<std_msgs::msg::String>::value && has_bounded_size<std_msgs::msg::UInt32>::value> {};

template<>
struct is_message<ros_gz_interfaces::msg::JointWrench>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__TRAITS_HPP_
