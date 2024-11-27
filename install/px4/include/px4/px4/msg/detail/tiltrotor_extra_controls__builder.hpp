// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/TiltrotorExtraControls.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__TILTROTOR_EXTRA_CONTROLS__BUILDER_HPP_
#define PX4__MSG__DETAIL__TILTROTOR_EXTRA_CONTROLS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/tiltrotor_extra_controls__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_TiltrotorExtraControls_collective_thrust_normalized_setpoint
{
public:
  explicit Init_TiltrotorExtraControls_collective_thrust_normalized_setpoint(::px4::msg::TiltrotorExtraControls & msg)
  : msg_(msg)
  {}
  ::px4::msg::TiltrotorExtraControls collective_thrust_normalized_setpoint(::px4::msg::TiltrotorExtraControls::_collective_thrust_normalized_setpoint_type arg)
  {
    msg_.collective_thrust_normalized_setpoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::TiltrotorExtraControls msg_;
};

class Init_TiltrotorExtraControls_collective_tilt_normalized_setpoint
{
public:
  explicit Init_TiltrotorExtraControls_collective_tilt_normalized_setpoint(::px4::msg::TiltrotorExtraControls & msg)
  : msg_(msg)
  {}
  Init_TiltrotorExtraControls_collective_thrust_normalized_setpoint collective_tilt_normalized_setpoint(::px4::msg::TiltrotorExtraControls::_collective_tilt_normalized_setpoint_type arg)
  {
    msg_.collective_tilt_normalized_setpoint = std::move(arg);
    return Init_TiltrotorExtraControls_collective_thrust_normalized_setpoint(msg_);
  }

private:
  ::px4::msg::TiltrotorExtraControls msg_;
};

class Init_TiltrotorExtraControls_timestamp
{
public:
  Init_TiltrotorExtraControls_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TiltrotorExtraControls_collective_tilt_normalized_setpoint timestamp(::px4::msg::TiltrotorExtraControls::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_TiltrotorExtraControls_collective_tilt_normalized_setpoint(msg_);
  }

private:
  ::px4::msg::TiltrotorExtraControls msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::TiltrotorExtraControls>()
{
  return px4::msg::builder::Init_TiltrotorExtraControls_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__TILTROTOR_EXTRA_CONTROLS__BUILDER_HPP_
