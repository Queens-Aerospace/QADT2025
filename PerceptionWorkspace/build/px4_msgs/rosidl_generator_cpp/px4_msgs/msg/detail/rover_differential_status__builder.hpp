// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverDifferentialStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_STATUS__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_differential_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverDifferentialStatus_pid_throttle_integral
{
public:
  explicit Init_RoverDifferentialStatus_pid_throttle_integral(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverDifferentialStatus pid_throttle_integral(::px4_msgs::msg::RoverDifferentialStatus::_pid_throttle_integral_type arg)
  {
    msg_.pid_throttle_integral = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_pid_yaw_rate_integral
{
public:
  explicit Init_RoverDifferentialStatus_pid_yaw_rate_integral(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_pid_throttle_integral pid_yaw_rate_integral(::px4_msgs::msg::RoverDifferentialStatus::_pid_yaw_rate_integral_type arg)
  {
    msg_.pid_yaw_rate_integral = std::move(arg);
    return Init_RoverDifferentialStatus_pid_throttle_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_pid_yaw_integral
{
public:
  explicit Init_RoverDifferentialStatus_pid_yaw_integral(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_pid_yaw_rate_integral pid_yaw_integral(::px4_msgs::msg::RoverDifferentialStatus::_pid_yaw_integral_type arg)
  {
    msg_.pid_yaw_integral = std::move(arg);
    return Init_RoverDifferentialStatus_pid_yaw_rate_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_adjusted_yaw_rate_setpoint
{
public:
  explicit Init_RoverDifferentialStatus_adjusted_yaw_rate_setpoint(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_pid_yaw_integral adjusted_yaw_rate_setpoint(::px4_msgs::msg::RoverDifferentialStatus::_adjusted_yaw_rate_setpoint_type arg)
  {
    msg_.adjusted_yaw_rate_setpoint = std::move(arg);
    return Init_RoverDifferentialStatus_pid_yaw_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_measured_yaw_rate
{
public:
  explicit Init_RoverDifferentialStatus_measured_yaw_rate(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_adjusted_yaw_rate_setpoint measured_yaw_rate(::px4_msgs::msg::RoverDifferentialStatus::_measured_yaw_rate_type arg)
  {
    msg_.measured_yaw_rate = std::move(arg);
    return Init_RoverDifferentialStatus_adjusted_yaw_rate_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_clyaw_yaw_rate_setpoint
{
public:
  explicit Init_RoverDifferentialStatus_clyaw_yaw_rate_setpoint(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_measured_yaw_rate clyaw_yaw_rate_setpoint(::px4_msgs::msg::RoverDifferentialStatus::_clyaw_yaw_rate_setpoint_type arg)
  {
    msg_.clyaw_yaw_rate_setpoint = std::move(arg);
    return Init_RoverDifferentialStatus_measured_yaw_rate(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_adjusted_yaw_setpoint
{
public:
  explicit Init_RoverDifferentialStatus_adjusted_yaw_setpoint(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_clyaw_yaw_rate_setpoint adjusted_yaw_setpoint(::px4_msgs::msg::RoverDifferentialStatus::_adjusted_yaw_setpoint_type arg)
  {
    msg_.adjusted_yaw_setpoint = std::move(arg);
    return Init_RoverDifferentialStatus_clyaw_yaw_rate_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_measured_yaw
{
public:
  explicit Init_RoverDifferentialStatus_measured_yaw(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_adjusted_yaw_setpoint measured_yaw(::px4_msgs::msg::RoverDifferentialStatus::_measured_yaw_type arg)
  {
    msg_.measured_yaw = std::move(arg);
    return Init_RoverDifferentialStatus_adjusted_yaw_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_adjusted_forward_speed_setpoint
{
public:
  explicit Init_RoverDifferentialStatus_adjusted_forward_speed_setpoint(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_measured_yaw adjusted_forward_speed_setpoint(::px4_msgs::msg::RoverDifferentialStatus::_adjusted_forward_speed_setpoint_type arg)
  {
    msg_.adjusted_forward_speed_setpoint = std::move(arg);
    return Init_RoverDifferentialStatus_measured_yaw(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_measured_forward_speed
{
public:
  explicit Init_RoverDifferentialStatus_measured_forward_speed(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_adjusted_forward_speed_setpoint measured_forward_speed(::px4_msgs::msg::RoverDifferentialStatus::_measured_forward_speed_type arg)
  {
    msg_.measured_forward_speed = std::move(arg);
    return Init_RoverDifferentialStatus_adjusted_forward_speed_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_timestamp
{
public:
  Init_RoverDifferentialStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverDifferentialStatus_measured_forward_speed timestamp(::px4_msgs::msg::RoverDifferentialStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverDifferentialStatus_measured_forward_speed(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverDifferentialStatus>()
{
  return px4_msgs::msg::builder::Init_RoverDifferentialStatus_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_STATUS__BUILDER_HPP_
