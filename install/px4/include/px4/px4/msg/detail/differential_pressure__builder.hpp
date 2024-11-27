// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/DifferentialPressure.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__DIFFERENTIAL_PRESSURE__BUILDER_HPP_
#define PX4__MSG__DETAIL__DIFFERENTIAL_PRESSURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/differential_pressure__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_DifferentialPressure_error_count
{
public:
  explicit Init_DifferentialPressure_error_count(::px4::msg::DifferentialPressure & msg)
  : msg_(msg)
  {}
  ::px4::msg::DifferentialPressure error_count(::px4::msg::DifferentialPressure::_error_count_type arg)
  {
    msg_.error_count = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::DifferentialPressure msg_;
};

class Init_DifferentialPressure_temperature
{
public:
  explicit Init_DifferentialPressure_temperature(::px4::msg::DifferentialPressure & msg)
  : msg_(msg)
  {}
  Init_DifferentialPressure_error_count temperature(::px4::msg::DifferentialPressure::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_DifferentialPressure_error_count(msg_);
  }

private:
  ::px4::msg::DifferentialPressure msg_;
};

class Init_DifferentialPressure_differential_pressure_pa
{
public:
  explicit Init_DifferentialPressure_differential_pressure_pa(::px4::msg::DifferentialPressure & msg)
  : msg_(msg)
  {}
  Init_DifferentialPressure_temperature differential_pressure_pa(::px4::msg::DifferentialPressure::_differential_pressure_pa_type arg)
  {
    msg_.differential_pressure_pa = std::move(arg);
    return Init_DifferentialPressure_temperature(msg_);
  }

private:
  ::px4::msg::DifferentialPressure msg_;
};

class Init_DifferentialPressure_device_id
{
public:
  explicit Init_DifferentialPressure_device_id(::px4::msg::DifferentialPressure & msg)
  : msg_(msg)
  {}
  Init_DifferentialPressure_differential_pressure_pa device_id(::px4::msg::DifferentialPressure::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_DifferentialPressure_differential_pressure_pa(msg_);
  }

private:
  ::px4::msg::DifferentialPressure msg_;
};

class Init_DifferentialPressure_timestamp_sample
{
public:
  explicit Init_DifferentialPressure_timestamp_sample(::px4::msg::DifferentialPressure & msg)
  : msg_(msg)
  {}
  Init_DifferentialPressure_device_id timestamp_sample(::px4::msg::DifferentialPressure::_timestamp_sample_type arg)
  {
    msg_.timestamp_sample = std::move(arg);
    return Init_DifferentialPressure_device_id(msg_);
  }

private:
  ::px4::msg::DifferentialPressure msg_;
};

class Init_DifferentialPressure_timestamp
{
public:
  Init_DifferentialPressure_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DifferentialPressure_timestamp_sample timestamp(::px4::msg::DifferentialPressure::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_DifferentialPressure_timestamp_sample(msg_);
  }

private:
  ::px4::msg::DifferentialPressure msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::DifferentialPressure>()
{
  return px4::msg::builder::Init_DifferentialPressure_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__DIFFERENTIAL_PRESSURE__BUILDER_HPP_
