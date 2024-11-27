// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/GpioIn.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__GPIO_IN__BUILDER_HPP_
#define PX4__MSG__DETAIL__GPIO_IN__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/gpio_in__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_GpioIn_state
{
public:
  explicit Init_GpioIn_state(::px4::msg::GpioIn & msg)
  : msg_(msg)
  {}
  ::px4::msg::GpioIn state(::px4::msg::GpioIn::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::GpioIn msg_;
};

class Init_GpioIn_device_id
{
public:
  explicit Init_GpioIn_device_id(::px4::msg::GpioIn & msg)
  : msg_(msg)
  {}
  Init_GpioIn_state device_id(::px4::msg::GpioIn::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_GpioIn_state(msg_);
  }

private:
  ::px4::msg::GpioIn msg_;
};

class Init_GpioIn_timestamp
{
public:
  Init_GpioIn_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpioIn_device_id timestamp(::px4::msg::GpioIn::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_GpioIn_device_id(msg_);
  }

private:
  ::px4::msg::GpioIn msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::GpioIn>()
{
  return px4::msg::builder::Init_GpioIn_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__GPIO_IN__BUILDER_HPP_
