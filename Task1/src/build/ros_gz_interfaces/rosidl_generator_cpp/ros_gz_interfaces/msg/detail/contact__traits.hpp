// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ros_gz_interfaces:msg/Contact.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__CONTACT__TRAITS_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__CONTACT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ros_gz_interfaces/msg/detail/contact__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'collision1'
// Member 'collision2'
#include "ros_gz_interfaces/msg/detail/entity__traits.hpp"
// Member 'positions'
// Member 'normals'
#include "geometry_msgs/msg/detail/vector3__traits.hpp"
// Member 'wrenches'
#include "ros_gz_interfaces/msg/detail/joint_wrench__traits.hpp"

namespace ros_gz_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const Contact & msg,
  std::ostream & out)
{
  out << "{";
  // member: collision1
  {
    out << "collision1: ";
    to_flow_style_yaml(msg.collision1, out);
    out << ", ";
  }

  // member: collision2
  {
    out << "collision2: ";
    to_flow_style_yaml(msg.collision2, out);
    out << ", ";
  }

  // member: positions
  {
    if (msg.positions.size() == 0) {
      out << "positions: []";
    } else {
      out << "positions: [";
      size_t pending_items = msg.positions.size();
      for (auto item : msg.positions) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: normals
  {
    if (msg.normals.size() == 0) {
      out << "normals: []";
    } else {
      out << "normals: [";
      size_t pending_items = msg.normals.size();
      for (auto item : msg.normals) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: depths
  {
    if (msg.depths.size() == 0) {
      out << "depths: []";
    } else {
      out << "depths: [";
      size_t pending_items = msg.depths.size();
      for (auto item : msg.depths) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: wrenches
  {
    if (msg.wrenches.size() == 0) {
      out << "wrenches: []";
    } else {
      out << "wrenches: [";
      size_t pending_items = msg.wrenches.size();
      for (auto item : msg.wrenches) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Contact & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: collision1
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision1:\n";
    to_block_style_yaml(msg.collision1, out, indentation + 2);
  }

  // member: collision2
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "collision2:\n";
    to_block_style_yaml(msg.collision2, out, indentation + 2);
  }

  // member: positions
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.positions.size() == 0) {
      out << "positions: []\n";
    } else {
      out << "positions:\n";
      for (auto item : msg.positions) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: normals
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.normals.size() == 0) {
      out << "normals: []\n";
    } else {
      out << "normals:\n";
      for (auto item : msg.normals) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }

  // member: depths
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.depths.size() == 0) {
      out << "depths: []\n";
    } else {
      out << "depths:\n";
      for (auto item : msg.depths) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: wrenches
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.wrenches.size() == 0) {
      out << "wrenches: []\n";
    } else {
      out << "wrenches:\n";
      for (auto item : msg.wrenches) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Contact & msg, bool use_flow_style = false)
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
  const ros_gz_interfaces::msg::Contact & msg,
  std::ostream & out, size_t indentation = 0)
{
  ros_gz_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ros_gz_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const ros_gz_interfaces::msg::Contact & msg)
{
  return ros_gz_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ros_gz_interfaces::msg::Contact>()
{
  return "ros_gz_interfaces::msg::Contact";
}

template<>
inline const char * name<ros_gz_interfaces::msg::Contact>()
{
  return "ros_gz_interfaces/msg/Contact";
}

template<>
struct has_fixed_size<ros_gz_interfaces::msg::Contact>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ros_gz_interfaces::msg::Contact>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ros_gz_interfaces::msg::Contact>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__CONTACT__TRAITS_HPP_
