// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/PpsCapture.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__PPS_CAPTURE__BUILDER_HPP_
#define PX4__MSG__DETAIL__PPS_CAPTURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/pps_capture__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_PpsCapture_pps_rate_exceeded_counter
{
public:
  explicit Init_PpsCapture_pps_rate_exceeded_counter(::px4::msg::PpsCapture & msg)
  : msg_(msg)
  {}
  ::px4::msg::PpsCapture pps_rate_exceeded_counter(::px4::msg::PpsCapture::_pps_rate_exceeded_counter_type arg)
  {
    msg_.pps_rate_exceeded_counter = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::PpsCapture msg_;
};

class Init_PpsCapture_rtc_timestamp
{
public:
  explicit Init_PpsCapture_rtc_timestamp(::px4::msg::PpsCapture & msg)
  : msg_(msg)
  {}
  Init_PpsCapture_pps_rate_exceeded_counter rtc_timestamp(::px4::msg::PpsCapture::_rtc_timestamp_type arg)
  {
    msg_.rtc_timestamp = std::move(arg);
    return Init_PpsCapture_pps_rate_exceeded_counter(msg_);
  }

private:
  ::px4::msg::PpsCapture msg_;
};

class Init_PpsCapture_timestamp
{
public:
  Init_PpsCapture_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PpsCapture_rtc_timestamp timestamp(::px4::msg::PpsCapture::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_PpsCapture_rtc_timestamp(msg_);
  }

private:
  ::px4::msg::PpsCapture msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::PpsCapture>()
{
  return px4::msg::builder::Init_PpsCapture_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__PPS_CAPTURE__BUILDER_HPP_
