// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/SensorAirflow.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__SENSOR_AIRFLOW__BUILDER_HPP_
#define PX4__MSG__DETAIL__SENSOR_AIRFLOW__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/sensor_airflow__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_SensorAirflow_status
{
public:
  explicit Init_SensorAirflow_status(::px4::msg::SensorAirflow & msg)
  : msg_(msg)
  {}
  ::px4::msg::SensorAirflow status(::px4::msg::SensorAirflow::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::SensorAirflow msg_;
};

class Init_SensorAirflow_direction
{
public:
  explicit Init_SensorAirflow_direction(::px4::msg::SensorAirflow & msg)
  : msg_(msg)
  {}
  Init_SensorAirflow_status direction(::px4::msg::SensorAirflow::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_SensorAirflow_status(msg_);
  }

private:
  ::px4::msg::SensorAirflow msg_;
};

class Init_SensorAirflow_speed
{
public:
  explicit Init_SensorAirflow_speed(::px4::msg::SensorAirflow & msg)
  : msg_(msg)
  {}
  Init_SensorAirflow_direction speed(::px4::msg::SensorAirflow::_speed_type arg)
  {
    msg_.speed = std::move(arg);
    return Init_SensorAirflow_direction(msg_);
  }

private:
  ::px4::msg::SensorAirflow msg_;
};

class Init_SensorAirflow_device_id
{
public:
  explicit Init_SensorAirflow_device_id(::px4::msg::SensorAirflow & msg)
  : msg_(msg)
  {}
  Init_SensorAirflow_speed device_id(::px4::msg::SensorAirflow::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_SensorAirflow_speed(msg_);
  }

private:
  ::px4::msg::SensorAirflow msg_;
};

class Init_SensorAirflow_timestamp
{
public:
  Init_SensorAirflow_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorAirflow_device_id timestamp(::px4::msg::SensorAirflow::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_SensorAirflow_device_id(msg_);
  }

private:
  ::px4::msg::SensorAirflow msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::SensorAirflow>()
{
  return px4::msg::builder::Init_SensorAirflow_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__SENSOR_AIRFLOW__BUILDER_HPP_
