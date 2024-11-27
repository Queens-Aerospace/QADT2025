// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/SensorPreflightMag.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__SENSOR_PREFLIGHT_MAG__BUILDER_HPP_
#define PX4__MSG__DETAIL__SENSOR_PREFLIGHT_MAG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/sensor_preflight_mag__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_SensorPreflightMag_mag_inconsistency_angle
{
public:
  explicit Init_SensorPreflightMag_mag_inconsistency_angle(::px4::msg::SensorPreflightMag & msg)
  : msg_(msg)
  {}
  ::px4::msg::SensorPreflightMag mag_inconsistency_angle(::px4::msg::SensorPreflightMag::_mag_inconsistency_angle_type arg)
  {
    msg_.mag_inconsistency_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::SensorPreflightMag msg_;
};

class Init_SensorPreflightMag_timestamp
{
public:
  Init_SensorPreflightMag_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorPreflightMag_mag_inconsistency_angle timestamp(::px4::msg::SensorPreflightMag::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SensorPreflightMag_mag_inconsistency_angle(msg_);
  }

private:
  ::px4::msg::SensorPreflightMag msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::SensorPreflightMag>()
{
  return px4::msg::builder::Init_SensorPreflightMag_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__SENSOR_PREFLIGHT_MAG__BUILDER_HPP_
