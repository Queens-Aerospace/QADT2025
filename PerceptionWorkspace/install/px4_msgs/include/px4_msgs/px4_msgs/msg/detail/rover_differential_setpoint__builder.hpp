// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverDifferentialSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_differential_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverDifferentialSetpoint_yaw_setpoint
{
public:
  explicit Init_RoverDifferentialSetpoint_yaw_setpoint(::px4_msgs::msg::RoverDifferentialSetpoint & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverDifferentialSetpoint yaw_setpoint(::px4_msgs::msg::RoverDifferentialSetpoint::_yaw_setpoint_type arg)
  {
    msg_.yaw_setpoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialSetpoint msg_;
};

class Init_RoverDifferentialSetpoint_speed_diff_setpoint_normalized
{
public:
  explicit Init_RoverDifferentialSetpoint_speed_diff_setpoint_normalized(::px4_msgs::msg::RoverDifferentialSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialSetpoint_yaw_setpoint speed_diff_setpoint_normalized(::px4_msgs::msg::RoverDifferentialSetpoint::_speed_diff_setpoint_normalized_type arg)
  {
    msg_.speed_diff_setpoint_normalized = std::move(arg);
    return Init_RoverDifferentialSetpoint_yaw_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialSetpoint msg_;
};

class Init_RoverDifferentialSetpoint_yaw_rate_setpoint
{
public:
  explicit Init_RoverDifferentialSetpoint_yaw_rate_setpoint(::px4_msgs::msg::RoverDifferentialSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialSetpoint_speed_diff_setpoint_normalized yaw_rate_setpoint(::px4_msgs::msg::RoverDifferentialSetpoint::_yaw_rate_setpoint_type arg)
  {
    msg_.yaw_rate_setpoint = std::move(arg);
    return Init_RoverDifferentialSetpoint_speed_diff_setpoint_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialSetpoint msg_;
};

class Init_RoverDifferentialSetpoint_forward_speed_setpoint_normalized
{
public:
  explicit Init_RoverDifferentialSetpoint_forward_speed_setpoint_normalized(::px4_msgs::msg::RoverDifferentialSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialSetpoint_yaw_rate_setpoint forward_speed_setpoint_normalized(::px4_msgs::msg::RoverDifferentialSetpoint::_forward_speed_setpoint_normalized_type arg)
  {
    msg_.forward_speed_setpoint_normalized = std::move(arg);
    return Init_RoverDifferentialSetpoint_yaw_rate_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialSetpoint msg_;
};

class Init_RoverDifferentialSetpoint_forward_speed_setpoint
{
public:
  explicit Init_RoverDifferentialSetpoint_forward_speed_setpoint(::px4_msgs::msg::RoverDifferentialSetpoint & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialSetpoint_forward_speed_setpoint_normalized forward_speed_setpoint(::px4_msgs::msg::RoverDifferentialSetpoint::_forward_speed_setpoint_type arg)
  {
    msg_.forward_speed_setpoint = std::move(arg);
    return Init_RoverDifferentialSetpoint_forward_speed_setpoint_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialSetpoint msg_;
};

class Init_RoverDifferentialSetpoint_timestamp
{
public:
  Init_RoverDifferentialSetpoint_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverDifferentialSetpoint_forward_speed_setpoint timestamp(::px4_msgs::msg::RoverDifferentialSetpoint::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverDifferentialSetpoint_forward_speed_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverDifferentialSetpoint>()
{
  return px4_msgs::msg::builder::Init_RoverDifferentialSetpoint_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_SETPOINT__BUILDER_HPP_
