// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/DebugValue.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__DEBUG_VALUE__BUILDER_HPP_
#define PX4__MSG__DETAIL__DEBUG_VALUE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/debug_value__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_DebugValue_value
{
public:
  explicit Init_DebugValue_value(::px4::msg::DebugValue & msg)
  : msg_(msg)
  {}
  ::px4::msg::DebugValue value(::px4::msg::DebugValue::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::DebugValue msg_;
};

class Init_DebugValue_ind
{
public:
  explicit Init_DebugValue_ind(::px4::msg::DebugValue & msg)
  : msg_(msg)
  {}
  Init_DebugValue_value ind(::px4::msg::DebugValue::_ind_type arg)
  {
    msg_.ind = std::move(arg);
    return Init_DebugValue_value(msg_);
  }

private:
  ::px4::msg::DebugValue msg_;
};

class Init_DebugValue_timestamp
{
public:
  Init_DebugValue_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugValue_ind timestamp(::px4::msg::DebugValue::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_DebugValue_ind(msg_);
  }

private:
  ::px4::msg::DebugValue msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::DebugValue>()
{
  return px4::msg::builder::Init_DebugValue_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__DEBUG_VALUE__BUILDER_HPP_
