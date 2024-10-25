// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverAckermannStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_ackermann_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverAckermannStatus_actual_speed
{
public:
  explicit Init_RoverAckermannStatus_actual_speed(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverAckermannStatus actual_speed(::px4_msgs::msg::RoverAckermannStatus::_actual_speed_type arg)
  {
    msg_.actual_speed = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_steering_setpoint
{
public:
  explicit Init_RoverAckermannStatus_steering_setpoint(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_actual_speed steering_setpoint(::px4_msgs::msg::RoverAckermannStatus::_steering_setpoint_type arg)
  {
    msg_.steering_setpoint = std::move(arg);
    return Init_RoverAckermannStatus_actual_speed(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_throttle_setpoint
{
public:
  explicit Init_RoverAckermannStatus_throttle_setpoint(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_steering_setpoint throttle_setpoint(::px4_msgs::msg::RoverAckermannStatus::_throttle_setpoint_type arg)
  {
    msg_.throttle_setpoint = std::move(arg);
    return Init_RoverAckermannStatus_steering_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_timestamp
{
public:
  Init_RoverAckermannStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverAckermannStatus_throttle_setpoint timestamp(::px4_msgs::msg::RoverAckermannStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverAckermannStatus_throttle_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverAckermannStatus>()
{
  return px4_msgs::msg::builder::Init_RoverAckermannStatus_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__BUILDER_HPP_
