// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/PositionSetpointTriplet.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__POSITION_SETPOINT_TRIPLET__BUILDER_HPP_
#define PX4__MSG__DETAIL__POSITION_SETPOINT_TRIPLET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/position_setpoint_triplet__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_PositionSetpointTriplet_next
{
public:
  explicit Init_PositionSetpointTriplet_next(::px4::msg::PositionSetpointTriplet & msg)
  : msg_(msg)
  {}
  ::px4::msg::PositionSetpointTriplet next(::px4::msg::PositionSetpointTriplet::_next_type arg)
  {
    msg_.next = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::PositionSetpointTriplet msg_;
};

class Init_PositionSetpointTriplet_current
{
public:
  explicit Init_PositionSetpointTriplet_current(::px4::msg::PositionSetpointTriplet & msg)
  : msg_(msg)
  {}
  Init_PositionSetpointTriplet_next current(::px4::msg::PositionSetpointTriplet::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_PositionSetpointTriplet_next(msg_);
  }

private:
  ::px4::msg::PositionSetpointTriplet msg_;
};

class Init_PositionSetpointTriplet_previous
{
public:
  explicit Init_PositionSetpointTriplet_previous(::px4::msg::PositionSetpointTriplet & msg)
  : msg_(msg)
  {}
  Init_PositionSetpointTriplet_current previous(::px4::msg::PositionSetpointTriplet::_previous_type arg)
  {
    msg_.previous = std::move(arg);
    return Init_PositionSetpointTriplet_current(msg_);
  }

private:
  ::px4::msg::PositionSetpointTriplet msg_;
};

class Init_PositionSetpointTriplet_timestamp
{
public:
  Init_PositionSetpointTriplet_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionSetpointTriplet_previous timestamp(::px4::msg::PositionSetpointTriplet::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_PositionSetpointTriplet_previous(msg_);
  }

private:
  ::px4::msg::PositionSetpointTriplet msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::PositionSetpointTriplet>()
{
  return px4::msg::builder::Init_PositionSetpointTriplet_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__POSITION_SETPOINT_TRIPLET__BUILDER_HPP_
