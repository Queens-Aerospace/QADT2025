// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/ActuatorMotors.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ACTUATOR_MOTORS__BUILDER_HPP_
#define PX4__MSG__DETAIL__ACTUATOR_MOTORS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/actuator_motors__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_ActuatorMotors_control
{
public:
  explicit Init_ActuatorMotors_control(::px4::msg::ActuatorMotors & msg)
  : msg_(msg)
  {}
  ::px4::msg::ActuatorMotors control(::px4::msg::ActuatorMotors::_control_type arg)
  {
    msg_.control = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::ActuatorMotors msg_;
};

class Init_ActuatorMotors_reversible_flags
{
public:
  explicit Init_ActuatorMotors_reversible_flags(::px4::msg::ActuatorMotors & msg)
  : msg_(msg)
  {}
  Init_ActuatorMotors_control reversible_flags(::px4::msg::ActuatorMotors::_reversible_flags_type arg)
  {
    msg_.reversible_flags = std::move(arg);
    return Init_ActuatorMotors_control(msg_);
  }

private:
  ::px4::msg::ActuatorMotors msg_;
};

class Init_ActuatorMotors_timestamp_sample
{
public:
  explicit Init_ActuatorMotors_timestamp_sample(::px4::msg::ActuatorMotors & msg)
  : msg_(msg)
  {}
  Init_ActuatorMotors_reversible_flags timestamp_sample(::px4::msg::ActuatorMotors::_timestamp_sample_type arg)
  {
    msg_.timestamp_sample = std::move(arg);
    return Init_ActuatorMotors_reversible_flags(msg_);
  }

private:
  ::px4::msg::ActuatorMotors msg_;
};

class Init_ActuatorMotors_timestamp
{
public:
  Init_ActuatorMotors_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ActuatorMotors_timestamp_sample timestamp(::px4::msg::ActuatorMotors::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_ActuatorMotors_timestamp_sample(msg_);
  }

private:
  ::px4::msg::ActuatorMotors msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::ActuatorMotors>()
{
  return px4::msg::builder::Init_ActuatorMotors_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ACTUATOR_MOTORS__BUILDER_HPP_
