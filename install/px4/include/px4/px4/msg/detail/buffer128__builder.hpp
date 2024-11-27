// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/Buffer128.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__BUFFER128__BUILDER_HPP_
#define PX4__MSG__DETAIL__BUFFER128__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/buffer128__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_Buffer128_data
{
public:
  explicit Init_Buffer128_data(::px4::msg::Buffer128 & msg)
  : msg_(msg)
  {}
  ::px4::msg::Buffer128 data(::px4::msg::Buffer128::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::Buffer128 msg_;
};

class Init_Buffer128_len
{
public:
  explicit Init_Buffer128_len(::px4::msg::Buffer128 & msg)
  : msg_(msg)
  {}
  Init_Buffer128_data len(::px4::msg::Buffer128::_len_type arg)
  {
    msg_.len = std::move(arg);
    return Init_Buffer128_data(msg_);
  }

private:
  ::px4::msg::Buffer128 msg_;
};

class Init_Buffer128_timestamp
{
public:
  Init_Buffer128_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Buffer128_len timestamp(::px4::msg::Buffer128::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Buffer128_len(msg_);
  }

private:
  ::px4::msg::Buffer128 msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::Buffer128>()
{
  return px4::msg::builder::Init_Buffer128_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__BUFFER128__BUILDER_HPP_
