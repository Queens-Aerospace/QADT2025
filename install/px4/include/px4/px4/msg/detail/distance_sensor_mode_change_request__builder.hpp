// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/DistanceSensorModeChangeRequest.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__DISTANCE_SENSOR_MODE_CHANGE_REQUEST__BUILDER_HPP_
#define PX4__MSG__DETAIL__DISTANCE_SENSOR_MODE_CHANGE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/distance_sensor_mode_change_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_DistanceSensorModeChangeRequest_request_on_off
{
public:
  explicit Init_DistanceSensorModeChangeRequest_request_on_off(::px4::msg::DistanceSensorModeChangeRequest & msg)
  : msg_(msg)
  {}
  ::px4::msg::DistanceSensorModeChangeRequest request_on_off(::px4::msg::DistanceSensorModeChangeRequest::_request_on_off_type arg)
  {
    msg_.request_on_off = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::DistanceSensorModeChangeRequest msg_;
};

class Init_DistanceSensorModeChangeRequest_timestamp
{
public:
  Init_DistanceSensorModeChangeRequest_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DistanceSensorModeChangeRequest_request_on_off timestamp(::px4::msg::DistanceSensorModeChangeRequest::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_DistanceSensorModeChangeRequest_request_on_off(msg_);
  }

private:
  ::px4::msg::DistanceSensorModeChangeRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::DistanceSensorModeChangeRequest>()
{
  return px4::msg::builder::Init_DistanceSensorModeChangeRequest_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__DISTANCE_SENSOR_MODE_CHANGE_REQUEST__BUILDER_HPP_
