// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/SensorAccel.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__SENSOR_ACCEL__BUILDER_HPP_
#define PX4__MSG__DETAIL__SENSOR_ACCEL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/sensor_accel__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_SensorAccel_samples
{
public:
  explicit Init_SensorAccel_samples(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  ::px4::msg::SensorAccel samples(::px4::msg::SensorAccel::_samples_type arg)
  {
    msg_.samples = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_clip_counter
{
public:
  explicit Init_SensorAccel_clip_counter(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_samples clip_counter(::px4::msg::SensorAccel::_clip_counter_type arg)
  {
    msg_.clip_counter = std::move(arg);
    return Init_SensorAccel_samples(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_error_count
{
public:
  explicit Init_SensorAccel_error_count(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_clip_counter error_count(::px4::msg::SensorAccel::_error_count_type arg)
  {
    msg_.error_count = std::move(arg);
    return Init_SensorAccel_clip_counter(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_temperature
{
public:
  explicit Init_SensorAccel_temperature(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_error_count temperature(::px4::msg::SensorAccel::_temperature_type arg)
  {
    msg_.temperature = std::move(arg);
    return Init_SensorAccel_error_count(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_z
{
public:
  explicit Init_SensorAccel_z(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_temperature z(::px4::msg::SensorAccel::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_SensorAccel_temperature(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_y
{
public:
  explicit Init_SensorAccel_y(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_z y(::px4::msg::SensorAccel::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_SensorAccel_z(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_x
{
public:
  explicit Init_SensorAccel_x(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_y x(::px4::msg::SensorAccel::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_SensorAccel_y(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_device_id
{
public:
  explicit Init_SensorAccel_device_id(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_x device_id(::px4::msg::SensorAccel::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_SensorAccel_x(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_timestamp_sample
{
public:
  explicit Init_SensorAccel_timestamp_sample(::px4::msg::SensorAccel & msg)
  : msg_(msg)
  {}
  Init_SensorAccel_device_id timestamp_sample(::px4::msg::SensorAccel::_timestamp_sample_type arg)
  {
    msg_.timestamp_sample = std::move(arg);
    return Init_SensorAccel_device_id(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

class Init_SensorAccel_timestamp
{
public:
  Init_SensorAccel_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorAccel_timestamp_sample timestamp(::px4::msg::SensorAccel::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SensorAccel_timestamp_sample(msg_);
  }

private:
  ::px4::msg::SensorAccel msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::SensorAccel>()
{
  return px4::msg::builder::Init_SensorAccel_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__SENSOR_ACCEL__BUILDER_HPP_
