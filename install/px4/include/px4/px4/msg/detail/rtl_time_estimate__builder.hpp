// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/RtlTimeEstimate.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__RTL_TIME_ESTIMATE__BUILDER_HPP_
#define PX4__MSG__DETAIL__RTL_TIME_ESTIMATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/rtl_time_estimate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_RtlTimeEstimate_safe_time_estimate
{
public:
  explicit Init_RtlTimeEstimate_safe_time_estimate(::px4::msg::RtlTimeEstimate & msg)
  : msg_(msg)
  {}
  ::px4::msg::RtlTimeEstimate safe_time_estimate(::px4::msg::RtlTimeEstimate::_safe_time_estimate_type arg)
  {
    msg_.safe_time_estimate = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::RtlTimeEstimate msg_;
};

class Init_RtlTimeEstimate_time_estimate
{
public:
  explicit Init_RtlTimeEstimate_time_estimate(::px4::msg::RtlTimeEstimate & msg)
  : msg_(msg)
  {}
  Init_RtlTimeEstimate_safe_time_estimate time_estimate(::px4::msg::RtlTimeEstimate::_time_estimate_type arg)
  {
    msg_.time_estimate = std::move(arg);
    return Init_RtlTimeEstimate_safe_time_estimate(msg_);
  }

private:
  ::px4::msg::RtlTimeEstimate msg_;
};

class Init_RtlTimeEstimate_valid
{
public:
  explicit Init_RtlTimeEstimate_valid(::px4::msg::RtlTimeEstimate & msg)
  : msg_(msg)
  {}
  Init_RtlTimeEstimate_time_estimate valid(::px4::msg::RtlTimeEstimate::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_RtlTimeEstimate_time_estimate(msg_);
  }

private:
  ::px4::msg::RtlTimeEstimate msg_;
};

class Init_RtlTimeEstimate_timestamp
{
public:
  Init_RtlTimeEstimate_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RtlTimeEstimate_valid timestamp(::px4::msg::RtlTimeEstimate::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RtlTimeEstimate_valid(msg_);
  }

private:
  ::px4::msg::RtlTimeEstimate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::RtlTimeEstimate>()
{
  return px4::msg::builder::Init_RtlTimeEstimate_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__RTL_TIME_ESTIMATE__BUILDER_HPP_
