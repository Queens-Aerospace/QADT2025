// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/HealthReport.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__HEALTH_REPORT__BUILDER_HPP_
#define PX4__MSG__DETAIL__HEALTH_REPORT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/health_report__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_HealthReport_arming_check_error_flags
{
public:
  explicit Init_HealthReport_arming_check_error_flags(::px4::msg::HealthReport & msg)
  : msg_(msg)
  {}
  ::px4::msg::HealthReport arming_check_error_flags(::px4::msg::HealthReport::_arming_check_error_flags_type arg)
  {
    msg_.arming_check_error_flags = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

class Init_HealthReport_arming_check_warning_flags
{
public:
  explicit Init_HealthReport_arming_check_warning_flags(::px4::msg::HealthReport & msg)
  : msg_(msg)
  {}
  Init_HealthReport_arming_check_error_flags arming_check_warning_flags(::px4::msg::HealthReport::_arming_check_warning_flags_type arg)
  {
    msg_.arming_check_warning_flags = std::move(arg);
    return Init_HealthReport_arming_check_error_flags(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

class Init_HealthReport_health_error_flags
{
public:
  explicit Init_HealthReport_health_error_flags(::px4::msg::HealthReport & msg)
  : msg_(msg)
  {}
  Init_HealthReport_arming_check_warning_flags health_error_flags(::px4::msg::HealthReport::_health_error_flags_type arg)
  {
    msg_.health_error_flags = std::move(arg);
    return Init_HealthReport_arming_check_warning_flags(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

class Init_HealthReport_health_warning_flags
{
public:
  explicit Init_HealthReport_health_warning_flags(::px4::msg::HealthReport & msg)
  : msg_(msg)
  {}
  Init_HealthReport_health_error_flags health_warning_flags(::px4::msg::HealthReport::_health_warning_flags_type arg)
  {
    msg_.health_warning_flags = std::move(arg);
    return Init_HealthReport_health_error_flags(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

class Init_HealthReport_health_is_present_flags
{
public:
  explicit Init_HealthReport_health_is_present_flags(::px4::msg::HealthReport & msg)
  : msg_(msg)
  {}
  Init_HealthReport_health_warning_flags health_is_present_flags(::px4::msg::HealthReport::_health_is_present_flags_type arg)
  {
    msg_.health_is_present_flags = std::move(arg);
    return Init_HealthReport_health_warning_flags(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

class Init_HealthReport_can_run_mode_flags
{
public:
  explicit Init_HealthReport_can_run_mode_flags(::px4::msg::HealthReport & msg)
  : msg_(msg)
  {}
  Init_HealthReport_health_is_present_flags can_run_mode_flags(::px4::msg::HealthReport::_can_run_mode_flags_type arg)
  {
    msg_.can_run_mode_flags = std::move(arg);
    return Init_HealthReport_health_is_present_flags(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

class Init_HealthReport_can_arm_mode_flags
{
public:
  explicit Init_HealthReport_can_arm_mode_flags(::px4::msg::HealthReport & msg)
  : msg_(msg)
  {}
  Init_HealthReport_can_run_mode_flags can_arm_mode_flags(::px4::msg::HealthReport::_can_arm_mode_flags_type arg)
  {
    msg_.can_arm_mode_flags = std::move(arg);
    return Init_HealthReport_can_run_mode_flags(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

class Init_HealthReport_timestamp
{
public:
  Init_HealthReport_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_HealthReport_can_arm_mode_flags timestamp(::px4::msg::HealthReport::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_HealthReport_can_arm_mode_flags(msg_);
  }

private:
  ::px4::msg::HealthReport msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::HealthReport>()
{
  return px4::msg::builder::Init_HealthReport_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__HEALTH_REPORT__BUILDER_HPP_
