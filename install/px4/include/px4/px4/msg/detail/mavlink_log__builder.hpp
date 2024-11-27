// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/MavlinkLog.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MAVLINK_LOG__BUILDER_HPP_
#define PX4__MSG__DETAIL__MAVLINK_LOG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/mavlink_log__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_MavlinkLog_severity
{
public:
  explicit Init_MavlinkLog_severity(::px4::msg::MavlinkLog & msg)
  : msg_(msg)
  {}
  ::px4::msg::MavlinkLog severity(::px4::msg::MavlinkLog::_severity_type arg)
  {
    msg_.severity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::MavlinkLog msg_;
};

class Init_MavlinkLog_text
{
public:
  explicit Init_MavlinkLog_text(::px4::msg::MavlinkLog & msg)
  : msg_(msg)
  {}
  Init_MavlinkLog_severity text(::px4::msg::MavlinkLog::_text_type arg)
  {
    msg_.text = std::move(arg);
    return Init_MavlinkLog_severity(msg_);
  }

private:
  ::px4::msg::MavlinkLog msg_;
};

class Init_MavlinkLog_timestamp
{
public:
  Init_MavlinkLog_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MavlinkLog_text timestamp(::px4::msg::MavlinkLog::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_MavlinkLog_text(msg_);
  }

private:
  ::px4::msg::MavlinkLog msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::MavlinkLog>()
{
  return px4::msg::builder::Init_MavlinkLog_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__MAVLINK_LOG__BUILDER_HPP_
