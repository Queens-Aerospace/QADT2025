// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4:msg/CanInterfaceStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__CAN_INTERFACE_STATUS__TRAITS_HPP_
#define PX4__MSG__DETAIL__CAN_INTERFACE_STATUS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4/msg/detail/can_interface_status__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4
{

namespace msg
{

inline void to_flow_style_yaml(
  const CanInterfaceStatus & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: interface
  {
    out << "interface: ";
    rosidl_generator_traits::value_to_yaml(msg.interface, out);
    out << ", ";
  }

  // member: io_errors
  {
    out << "io_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.io_errors, out);
    out << ", ";
  }

  // member: frames_tx
  {
    out << "frames_tx: ";
    rosidl_generator_traits::value_to_yaml(msg.frames_tx, out);
    out << ", ";
  }

  // member: frames_rx
  {
    out << "frames_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.frames_rx, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const CanInterfaceStatus & msg,
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

  // member: interface
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "interface: ";
    rosidl_generator_traits::value_to_yaml(msg.interface, out);
    out << "\n";
  }

  // member: io_errors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "io_errors: ";
    rosidl_generator_traits::value_to_yaml(msg.io_errors, out);
    out << "\n";
  }

  // member: frames_tx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frames_tx: ";
    rosidl_generator_traits::value_to_yaml(msg.frames_tx, out);
    out << "\n";
  }

  // member: frames_rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "frames_rx: ";
    rosidl_generator_traits::value_to_yaml(msg.frames_rx, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const CanInterfaceStatus & msg, bool use_flow_style = false)
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

}  // namespace px4

namespace rosidl_generator_traits
{

[[deprecated("use px4::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const px4::msg::CanInterfaceStatus & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4::msg::CanInterfaceStatus & msg)
{
  return px4::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4::msg::CanInterfaceStatus>()
{
  return "px4::msg::CanInterfaceStatus";
}

template<>
inline const char * name<px4::msg::CanInterfaceStatus>()
{
  return "px4/msg/CanInterfaceStatus";
}

template<>
struct has_fixed_size<px4::msg::CanInterfaceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4::msg::CanInterfaceStatus>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4::msg::CanInterfaceStatus>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4__MSG__DETAIL__CAN_INTERFACE_STATUS__TRAITS_HPP_
