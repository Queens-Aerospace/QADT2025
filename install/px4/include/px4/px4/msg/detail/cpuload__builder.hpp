// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/Cpuload.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__CPULOAD__BUILDER_HPP_
#define PX4__MSG__DETAIL__CPULOAD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/cpuload__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_Cpuload_ram_usage
{
public:
  explicit Init_Cpuload_ram_usage(::px4::msg::Cpuload & msg)
  : msg_(msg)
  {}
  ::px4::msg::Cpuload ram_usage(::px4::msg::Cpuload::_ram_usage_type arg)
  {
    msg_.ram_usage = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::Cpuload msg_;
};

class Init_Cpuload_load
{
public:
  explicit Init_Cpuload_load(::px4::msg::Cpuload & msg)
  : msg_(msg)
  {}
  Init_Cpuload_ram_usage load(::px4::msg::Cpuload::_load_type arg)
  {
    msg_.load = std::move(arg);
    return Init_Cpuload_ram_usage(msg_);
  }

private:
  ::px4::msg::Cpuload msg_;
};

class Init_Cpuload_timestamp
{
public:
  Init_Cpuload_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Cpuload_load timestamp(::px4::msg::Cpuload::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Cpuload_load(msg_);
  }

private:
  ::px4::msg::Cpuload msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::Cpuload>()
{
  return px4::msg::builder::Init_Cpuload_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__CPULOAD__BUILDER_HPP_
