// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverAckermannSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_SETPOINT__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_ackermann_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverAckermannSetpoint_lateral_acceleration_setpoint
{
public:
  explicit Init_RoverAckermannSetpoint_lateral_acceleration_setpoint(::px4_msgs::msg::RoverAckermannSetpoint & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverAckermannSetpoint lateral_acceleration_setpoint(::px4_msgs::msg::RoverAckermannSetpoint::_lateral_acceleration_setpoint_type arg)
  {
    msg_.lateral_acceleration_setpoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannSetpoint msg_;
};

class Init_RoverAckermannSetpoint_steering_setpoint_normalized
{
public:
  explicit Init_RoverAckermannSetpoint_steering_setpoint_normalized(::px4_msgs::msg::RoverAckermannSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannSetpoint_lateral_acceleration_setpoint steering_setpoint_normalized(::px4_msgs::msg::RoverAckermannSetpoint::_steering_setpoint_normalized_type arg)
  {
    msg_.steering_setpoint_normalized = std::move(arg);
    return Init_RoverAckermannSetpoint_lateral_acceleration_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannSetpoint msg_;
};

class Init_RoverAckermannSetpoint_steering_setpoint
{
public:
  explicit Init_RoverAckermannSetpoint_steering_setpoint(::px4_msgs::msg::RoverAckermannSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannSetpoint_steering_setpoint_normalized steering_setpoint(::px4_msgs::msg::RoverAckermannSetpoint::_steering_setpoint_type arg)
  {
    msg_.steering_setpoint = std::move(arg);
    return Init_RoverAckermannSetpoint_steering_setpoint_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannSetpoint msg_;
};

class Init_RoverAckermannSetpoint_forward_speed_setpoint_normalized
{
public:
  explicit Init_RoverAckermannSetpoint_forward_speed_setpoint_normalized(::px4_msgs::msg::RoverAckermannSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannSetpoint_steering_setpoint forward_speed_setpoint_normalized(::px4_msgs::msg::RoverAckermannSetpoint::_forward_speed_setpoint_normalized_type arg)
  {
    msg_.forward_speed_setpoint_normalized = std::move(arg);
    return Init_RoverAckermannSetpoint_steering_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannSetpoint msg_;
};

class Init_RoverAckermannSetpoint_forward_speed_setpoint
{
public:
  explicit Init_RoverAckermannSetpoint_forward_speed_setpoint(::px4_msgs::msg::RoverAckermannSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannSetpoint_forward_speed_setpoint_normalized forward_speed_setpoint(::px4_msgs::msg::RoverAckermannSetpoint::_forward_speed_setpoint_type arg)
  {
    msg_.forward_speed_setpoint = std::move(arg);
    return Init_RoverAckermannSetpoint_forward_speed_setpoint_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannSetpoint msg_;
};

class Init_RoverAckermannSetpoint_timestamp
{
public:
  Init_RoverAckermannSetpoint_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverAckermannSetpoint_forward_speed_setpoint timestamp(::px4_msgs::msg::RoverAckermannSetpoint::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverAckermannSetpoint_forward_speed_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverAckermannSetpoint>()
{
  return px4_msgs::msg::builder::Init_RoverAckermannSetpoint_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_SETPOINT__BUILDER_HPP_
