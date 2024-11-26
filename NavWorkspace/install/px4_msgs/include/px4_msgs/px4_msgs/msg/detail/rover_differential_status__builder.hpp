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

class Init_RoverDifferentialStatus_speed_diff_normalized
{
public:
  explicit Init_RoverDifferentialStatus_speed_diff_normalized(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_pid_yaw_integral speed_diff_normalized(::px4_msgs::msg::RoverDifferentialStatus::_speed_diff_normalized_type arg)
  {
    msg_.speed_diff_normalized = std::move(arg);
    return Init_RoverDifferentialStatus_pid_yaw_integral(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_forward_speed_normalized
{
public:
  explicit Init_RoverDifferentialStatus_forward_speed_normalized(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_speed_diff_normalized forward_speed_normalized(::px4_msgs::msg::RoverDifferentialStatus::_forward_speed_normalized_type arg)
  {
    msg_.forward_speed_normalized = std::move(arg);
    return Init_RoverDifferentialStatus_speed_diff_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_desired_yaw_rate
{
public:
  explicit Init_RoverDifferentialStatus_desired_yaw_rate(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_forward_speed_normalized desired_yaw_rate(::px4_msgs::msg::RoverDifferentialStatus::_desired_yaw_rate_type arg)
  {
    msg_.desired_yaw_rate = std::move(arg);
    return Init_RoverDifferentialStatus_forward_speed_normalized(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_actual_yaw_rate
{
public:
  explicit Init_RoverDifferentialStatus_actual_yaw_rate(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_desired_yaw_rate actual_yaw_rate(::px4_msgs::msg::RoverDifferentialStatus::_actual_yaw_rate_type arg)
  {
    msg_.actual_yaw_rate = std::move(arg);
    return Init_RoverDifferentialStatus_desired_yaw_rate(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_actual_yaw
{
public:
  explicit Init_RoverDifferentialStatus_actual_yaw(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_actual_yaw_rate actual_yaw(::px4_msgs::msg::RoverDifferentialStatus::_actual_yaw_type arg)
  {
    msg_.actual_yaw = std::move(arg);
    return Init_RoverDifferentialStatus_actual_yaw_rate(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialStatus msg_;
};

class Init_RoverDifferentialStatus_actual_speed
{
public:
  explicit Init_RoverDifferentialStatus_actual_speed(::px4_msgs::msg::RoverDifferentialStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialStatus_actual_yaw actual_speed(::px4_msgs::msg::RoverDifferentialStatus::_actual_speed_type arg)
  {
    msg_.actual_speed = std::move(arg);
    return Init_RoverDifferentialStatus_actual_yaw(msg_);
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
  Init_RoverDifferentialStatus_actual_speed timestamp(::px4_msgs::msg::RoverDifferentialStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverDifferentialStatus_actual_speed(msg_);
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
