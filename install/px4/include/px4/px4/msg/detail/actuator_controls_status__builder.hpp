// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/ActuatorControlsStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ACTUATOR_CONTROLS_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__ACTUATOR_CONTROLS_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/actuator_controls_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_ActuatorControlsStatus_control_power
{
public:
  explicit Init_ActuatorControlsStatus_control_power(::px4::msg::ActuatorControlsStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::ActuatorControlsStatus control_power(::px4::msg::ActuatorControlsStatus::_control_power_type arg)
  {
    msg_.control_power = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::ActuatorControlsStatus msg_;
};

class Init_ActuatorControlsStatus_timestamp
{
public:
  Init_ActuatorControlsStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActuatorControlsStatus_control_power timestamp(::px4::msg::ActuatorControlsStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_ActuatorControlsStatus_control_power(msg_);
  }

private:
  ::px4::msg::ActuatorControlsStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::ActuatorControlsStatus>()
{
  return px4::msg::builder::Init_ActuatorControlsStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ACTUATOR_CONTROLS_STATUS__BUILDER_HPP_
