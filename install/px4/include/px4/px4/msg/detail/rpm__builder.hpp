// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/Rpm.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__RPM__BUILDER_HPP_
#define PX4__MSG__DETAIL__RPM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/rpm__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_Rpm_estimated_accurancy_rpm
{
public:
  explicit Init_Rpm_estimated_accurancy_rpm(::px4::msg::Rpm & msg)
  : msg_(msg)
  {}
  ::px4::msg::Rpm estimated_accurancy_rpm(::px4::msg::Rpm::_estimated_accurancy_rpm_type arg)
  {
    msg_.estimated_accurancy_rpm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::Rpm msg_;
};

class Init_Rpm_indicated_frequency_rpm
{
public:
  explicit Init_Rpm_indicated_frequency_rpm(::px4::msg::Rpm & msg)
  : msg_(msg)
  {}
  Init_Rpm_estimated_accurancy_rpm indicated_frequency_rpm(::px4::msg::Rpm::_indicated_frequency_rpm_type arg)
  {
    msg_.indicated_frequency_rpm = std::move(arg);
    return Init_Rpm_estimated_accurancy_rpm(msg_);
  }

private:
  ::px4::msg::Rpm msg_;
};

class Init_Rpm_timestamp
{
public:
  Init_Rpm_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Rpm_indicated_frequency_rpm timestamp(::px4::msg::Rpm::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Rpm_indicated_frequency_rpm(msg_);
  }

private:
  ::px4::msg::Rpm msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::Rpm>()
{
  return px4::msg::builder::Init_Rpm_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__RPM__BUILDER_HPP_
