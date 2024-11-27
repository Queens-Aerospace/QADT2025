// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/DebugArray.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__DEBUG_ARRAY__BUILDER_HPP_
#define PX4__MSG__DETAIL__DEBUG_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/debug_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_DebugArray_data
{
public:
  explicit Init_DebugArray_data(::px4::msg::DebugArray & msg)
  : msg_(msg)
  {}
  ::px4::msg::DebugArray data(::px4::msg::DebugArray::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::DebugArray msg_;
};

class Init_DebugArray_name
{
public:
  explicit Init_DebugArray_name(::px4::msg::DebugArray & msg)
  : msg_(msg)
  {}
  Init_DebugArray_data name(::px4::msg::DebugArray::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_DebugArray_data(msg_);
  }

private:
  ::px4::msg::DebugArray msg_;
};

class Init_DebugArray_id
{
public:
  explicit Init_DebugArray_id(::px4::msg::DebugArray & msg)
  : msg_(msg)
  {}
  Init_DebugArray_name id(::px4::msg::DebugArray::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_DebugArray_name(msg_);
  }

private:
  ::px4::msg::DebugArray msg_;
};

class Init_DebugArray_timestamp
{
public:
  Init_DebugArray_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DebugArray_id timestamp(::px4::msg::DebugArray::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_DebugArray_id(msg_);
  }

private:
  ::px4::msg::DebugArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::DebugArray>()
{
  return px4::msg::builder::Init_DebugArray_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__DEBUG_ARRAY__BUILDER_HPP_
