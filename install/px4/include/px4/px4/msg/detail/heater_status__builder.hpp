// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/HeaterStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__HEATER_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__HEATER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/heater_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_HeaterStatus_mode
{
public:
  explicit Init_HeaterStatus_mode(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::HeaterStatus mode(::px4::msg::HeaterStatus::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_feed_forward_value
{
public:
  explicit Init_HeaterStatus_feed_forward_value(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_mode feed_forward_value(::px4::msg::HeaterStatus::_feed_forward_value_type arg)
  {
    msg_.feed_forward_value = std::move(arg);
    return Init_HeaterStatus_mode(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_integrator_value
{
public:
  explicit Init_HeaterStatus_integrator_value(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_feed_forward_value integrator_value(::px4::msg::HeaterStatus::_integrator_value_type arg)
  {
    msg_.integrator_value = std::move(arg);
    return Init_HeaterStatus_feed_forward_value(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_proportional_value
{
public:
  explicit Init_HeaterStatus_proportional_value(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_integrator_value proportional_value(::px4::msg::HeaterStatus::_proportional_value_type arg)
  {
    msg_.proportional_value = std::move(arg);
    return Init_HeaterStatus_integrator_value(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_controller_time_on_usec
{
public:
  explicit Init_HeaterStatus_controller_time_on_usec(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_proportional_value controller_time_on_usec(::px4::msg::HeaterStatus::_controller_time_on_usec_type arg)
  {
    msg_.controller_time_on_usec = std::move(arg);
    return Init_HeaterStatus_proportional_value(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_controller_period_usec
{
public:
  explicit Init_HeaterStatus_controller_period_usec(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_controller_time_on_usec controller_period_usec(::px4::msg::HeaterStatus::_controller_period_usec_type arg)
  {
    msg_.controller_period_usec = std::move(arg);
    return Init_HeaterStatus_controller_time_on_usec(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_temperature_target
{
public:
  explicit Init_HeaterStatus_temperature_target(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_controller_period_usec temperature_target(::px4::msg::HeaterStatus::_temperature_target_type arg)
  {
    msg_.temperature_target = std::move(arg);
    return Init_HeaterStatus_controller_period_usec(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_temperature_sensor
{
public:
  explicit Init_HeaterStatus_temperature_sensor(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_temperature_target temperature_sensor(::px4::msg::HeaterStatus::_temperature_sensor_type arg)
  {
    msg_.temperature_sensor = std::move(arg);
    return Init_HeaterStatus_temperature_target(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_temperature_target_met
{
public:
  explicit Init_HeaterStatus_temperature_target_met(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_temperature_sensor temperature_target_met(::px4::msg::HeaterStatus::_temperature_target_met_type arg)
  {
    msg_.temperature_target_met = std::move(arg);
    return Init_HeaterStatus_temperature_sensor(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_heater_on
{
public:
  explicit Init_HeaterStatus_heater_on(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_temperature_target_met heater_on(::px4::msg::HeaterStatus::_heater_on_type arg)
  {
    msg_.heater_on = std::move(arg);
    return Init_HeaterStatus_temperature_target_met(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_device_id
{
public:
  explicit Init_HeaterStatus_device_id(::px4::msg::HeaterStatus & msg)
  : msg_(msg)
  {}
  Init_HeaterStatus_heater_on device_id(::px4::msg::HeaterStatus::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_HeaterStatus_heater_on(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

class Init_HeaterStatus_timestamp
{
public:
  Init_HeaterStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HeaterStatus_device_id timestamp(::px4::msg::HeaterStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_HeaterStatus_device_id(msg_);
  }

private:
  ::px4::msg::HeaterStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::HeaterStatus>()
{
  return px4::msg::builder::Init_HeaterStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__HEATER_STATUS__BUILDER_HPP_
