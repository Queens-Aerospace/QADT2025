// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from px4:msg/NormalizedUnsignedSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__NORMALIZED_UNSIGNED_SETPOINT__TRAITS_HPP_
#define PX4__MSG__DETAIL__NORMALIZED_UNSIGNED_SETPOINT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "px4/msg/detail/normalized_unsigned_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace px4
{

namespace msg
{

inline void to_flow_style_yaml(
  const NormalizedUnsignedSetpoint & msg,
  std::ostream & out)
{
  out << "{";
  // member: timestamp
  {
    out << "timestamp: ";
    rosidl_generator_traits::value_to_yaml(msg.timestamp, out);
    out << ", ";
  }

  // member: normalized_setpoint
  {
    out << "normalized_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.normalized_setpoint, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const NormalizedUnsignedSetpoint & msg,
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

  // member: normalized_setpoint
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "normalized_setpoint: ";
    rosidl_generator_traits::value_to_yaml(msg.normalized_setpoint, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const NormalizedUnsignedSetpoint & msg, bool use_flow_style = false)
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
  const px4::msg::NormalizedUnsignedSetpoint & msg,
  std::ostream & out, size_t indentation = 0)
{
  px4::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use px4::msg::to_yaml() instead")]]
inline std::string to_yaml(const px4::msg::NormalizedUnsignedSetpoint & msg)
{
  return px4::msg::to_yaml(msg);
}

template<>
inline const char * data_type<px4::msg::NormalizedUnsignedSetpoint>()
{
  return "px4::msg::NormalizedUnsignedSetpoint";
}

template<>
inline const char * name<px4::msg::NormalizedUnsignedSetpoint>()
{
  return "px4/msg/NormalizedUnsignedSetpoint";
}

template<>
struct has_fixed_size<px4::msg::NormalizedUnsignedSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<px4::msg::NormalizedUnsignedSetpoint>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<px4::msg::NormalizedUnsignedSetpoint>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // PX4__MSG__DETAIL__NORMALIZED_UNSIGNED_SETPOINT__TRAITS_HPP_
