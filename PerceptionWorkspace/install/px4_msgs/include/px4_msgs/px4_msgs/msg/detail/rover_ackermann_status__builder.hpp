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

class Init_RoverAckermannStatus_pid_lat_accel_integral
{
public:
  explicit Init_RoverAckermannStatus_pid_lat_accel_integral(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverAckermannStatus pid_lat_accel_integral(::px4_msgs::msg::RoverAckermannStatus::_pid_lat_accel_integral_type arg)
  {
    msg_.pid_lat_accel_integral = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_pid_throttle_integral
{
public:
  explicit Init_RoverAckermannStatus_pid_throttle_integral(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_pid_lat_accel_integral pid_throttle_integral(::px4_msgs::msg::RoverAckermannStatus::_pid_throttle_integral_type arg)
  {
    msg_.pid_throttle_integral = std::move(arg);
    return Init_RoverAckermannStatus_pid_lat_accel_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_measured_lateral_acceleration
{
public:
  explicit Init_RoverAckermannStatus_measured_lateral_acceleration(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_pid_throttle_integral measured_lateral_acceleration(::px4_msgs::msg::RoverAckermannStatus::_measured_lateral_acceleration_type arg)
  {
    msg_.measured_lateral_acceleration = std::move(arg);
    return Init_RoverAckermannStatus_pid_throttle_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_adjusted_steering_setpoint_normalized
{
public:
  explicit Init_RoverAckermannStatus_adjusted_steering_setpoint_normalized(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_measured_lateral_acceleration adjusted_steering_setpoint_normalized(::px4_msgs::msg::RoverAckermannStatus::_adjusted_steering_setpoint_normalized_type arg)
  {
    msg_.adjusted_steering_setpoint_normalized = std::move(arg);
    return Init_RoverAckermannStatus_measured_lateral_acceleration(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_steering_setpoint_normalized
{
public:
  explicit Init_RoverAckermannStatus_steering_setpoint_normalized(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_adjusted_steering_setpoint_normalized steering_setpoint_normalized(::px4_msgs::msg::RoverAckermannStatus::_steering_setpoint_normalized_type arg)
  {
    msg_.steering_setpoint_normalized = std::move(arg);
    return Init_RoverAckermannStatus_adjusted_steering_setpoint_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_adjusted_forward_speed_setpoint
{
public:
  explicit Init_RoverAckermannStatus_adjusted_forward_speed_setpoint(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_steering_setpoint_normalized adjusted_forward_speed_setpoint(::px4_msgs::msg::RoverAckermannStatus::_adjusted_forward_speed_setpoint_type arg)
  {
    msg_.adjusted_forward_speed_setpoint = std::move(arg);
    return Init_RoverAckermannStatus_steering_setpoint_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannStatus msg_;
};

class Init_RoverAckermannStatus_measured_forward_speed
{
public:
  explicit Init_RoverAckermannStatus_measured_forward_speed(::px4_msgs::msg::RoverAckermannStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannStatus_adjusted_forward_speed_setpoint measured_forward_speed(::px4_msgs::msg::RoverAckermannStatus::_measured_forward_speed_type arg)
  {
    msg_.measured_forward_speed = std::move(arg);
    return Init_RoverAckermannStatus_adjusted_forward_speed_setpoint(msg_);
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
  Init_RoverAckermannStatus_measured_forward_speed timestamp(::px4_msgs::msg::RoverAckermannStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverAckermannStatus_measured_forward_speed(msg_);
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
