// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/TakeoffStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__TAKEOFF_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__TAKEOFF_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/takeoff_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_TakeoffStatus_tilt_limit
{
public:
  explicit Init_TakeoffStatus_tilt_limit(::px4::msg::TakeoffStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::TakeoffStatus tilt_limit(::px4::msg::TakeoffStatus::_tilt_limit_type arg)
  {
    msg_.tilt_limit = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::TakeoffStatus msg_;
};

class Init_TakeoffStatus_takeoff_state
{
public:
  explicit Init_TakeoffStatus_takeoff_state(::px4::msg::TakeoffStatus & msg)
  : msg_(msg)
  {}
  Init_TakeoffStatus_tilt_limit takeoff_state(::px4::msg::TakeoffStatus::_takeoff_state_type arg)
  {
    msg_.takeoff_state = std::move(arg);
    return Init_TakeoffStatus_tilt_limit(msg_);
  }

private:
  ::px4::msg::TakeoffStatus msg_;
};

class Init_TakeoffStatus_timestamp
{
public:
  Init_TakeoffStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TakeoffStatus_takeoff_state timestamp(::px4::msg::TakeoffStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_TakeoffStatus_takeoff_state(msg_);
  }

private:
  ::px4::msg::TakeoffStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::TakeoffStatus>()
{
  return px4::msg::builder::Init_TakeoffStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__TAKEOFF_STATUS__BUILDER_HPP_
