// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverMecanumStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_mecanum_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverMecanumStatus_pid_lateral_throttle_integral
{
public:
  explicit Init_RoverMecanumStatus_pid_lateral_throttle_integral(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverMecanumStatus pid_lateral_throttle_integral(::px4_msgs::msg::RoverMecanumStatus::_pid_lateral_throttle_integral_type arg)
  {
    msg_.pid_lateral_throttle_integral = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_pid_forward_throttle_integral
{
public:
  explicit Init_RoverMecanumStatus_pid_forward_throttle_integral(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_pid_lateral_throttle_integral pid_forward_throttle_integral(::px4_msgs::msg::RoverMecanumStatus::_pid_forward_throttle_integral_type arg)
  {
    msg_.pid_forward_throttle_integral = std::move(arg);
    return Init_RoverMecanumStatus_pid_lateral_throttle_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_pid_yaw_integral
{
public:
  explicit Init_RoverMecanumStatus_pid_yaw_integral(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_pid_forward_throttle_integral pid_yaw_integral(::px4_msgs::msg::RoverMecanumStatus::_pid_yaw_integral_type arg)
  {
    msg_.pid_yaw_integral = std::move(arg);
    return Init_RoverMecanumStatus_pid_forward_throttle_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_pid_yaw_rate_integral
{
public:
  explicit Init_RoverMecanumStatus_pid_yaw_rate_integral(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_pid_yaw_integral pid_yaw_rate_integral(::px4_msgs::msg::RoverMecanumStatus::_pid_yaw_rate_integral_type arg)
  {
    msg_.pid_yaw_rate_integral = std::move(arg);
    return Init_RoverMecanumStatus_pid_yaw_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_measured_yaw
{
public:
  explicit Init_RoverMecanumStatus_measured_yaw(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_pid_yaw_rate_integral measured_yaw(::px4_msgs::msg::RoverMecanumStatus::_measured_yaw_type arg)
  {
    msg_.measured_yaw = std::move(arg);
    return Init_RoverMecanumStatus_pid_yaw_rate_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_measured_yaw_rate
{
public:
  explicit Init_RoverMecanumStatus_measured_yaw_rate(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_measured_yaw measured_yaw_rate(::px4_msgs::msg::RoverMecanumStatus::_measured_yaw_rate_type arg)
  {
    msg_.measured_yaw_rate = std::move(arg);
    return Init_RoverMecanumStatus_measured_yaw(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_adjusted_yaw_rate_setpoint
{
public:
  explicit Init_RoverMecanumStatus_adjusted_yaw_rate_setpoint(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_measured_yaw_rate adjusted_yaw_rate_setpoint(::px4_msgs::msg::RoverMecanumStatus::_adjusted_yaw_rate_setpoint_type arg)
  {
    msg_.adjusted_yaw_rate_setpoint = std::move(arg);
    return Init_RoverMecanumStatus_measured_yaw_rate(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_measured_lateral_speed
{
public:
  explicit Init_RoverMecanumStatus_measured_lateral_speed(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_adjusted_yaw_rate_setpoint measured_lateral_speed(::px4_msgs::msg::RoverMecanumStatus::_measured_lateral_speed_type arg)
  {
    msg_.measured_lateral_speed = std::move(arg);
    return Init_RoverMecanumStatus_adjusted_yaw_rate_setpoint(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_measured_forward_speed
{
public:
  explicit Init_RoverMecanumStatus_measured_forward_speed(::px4_msgs::msg::RoverMecanumStatus & msg)
  : msg_(msg)
  {}
  Init_RoverMecanumStatus_measured_lateral_speed measured_forward_speed(::px4_msgs::msg::RoverMecanumStatus::_measured_forward_speed_type arg)
  {
    msg_.measured_forward_speed = std::move(arg);
    return Init_RoverMecanumStatus_measured_lateral_speed(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

class Init_RoverMecanumStatus_timestamp
{
public:
  Init_RoverMecanumStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverMecanumStatus_measured_forward_speed timestamp(::px4_msgs::msg::RoverMecanumStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverMecanumStatus_measured_forward_speed(msg_);
  }

private:
  ::px4_msgs::msg::RoverMecanumStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverMecanumStatus>()
{
  return px4_msgs::msg::builder::Init_RoverMecanumStatus_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__BUILDER_HPP_
