// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/GpioConfig.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__GPIO_CONFIG__BUILDER_HPP_
#define PX4__MSG__DETAIL__GPIO_CONFIG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/gpio_config__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_GpioConfig_config
{
public:
  explicit Init_GpioConfig_config(::px4::msg::GpioConfig & msg)
  : msg_(msg)
  {}
  ::px4::msg::GpioConfig config(::px4::msg::GpioConfig::_config_type arg)
  {
    msg_.config = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::GpioConfig msg_;
};

class Init_GpioConfig_state
{
public:
  explicit Init_GpioConfig_state(::px4::msg::GpioConfig & msg)
  : msg_(msg)
  {}
  Init_GpioConfig_config state(::px4::msg::GpioConfig::_state_type arg)
  {
    msg_.state = std::move(arg);
    return Init_GpioConfig_config(msg_);
  }

private:
  ::px4::msg::GpioConfig msg_;
};

class Init_GpioConfig_mask
{
public:
  explicit Init_GpioConfig_mask(::px4::msg::GpioConfig & msg)
  : msg_(msg)
  {}
  Init_GpioConfig_state mask(::px4::msg::GpioConfig::_mask_type arg)
  {
    msg_.mask = std::move(arg);
    return Init_GpioConfig_state(msg_);
  }

private:
  ::px4::msg::GpioConfig msg_;
};

class Init_GpioConfig_device_id
{
public:
  explicit Init_GpioConfig_device_id(::px4::msg::GpioConfig & msg)
  : msg_(msg)
  {}
  Init_GpioConfig_mask device_id(::px4::msg::GpioConfig::_device_id_type arg)
  {
    msg_.device_id = std::move(arg);
    return Init_GpioConfig_mask(msg_);
  }

private:
  ::px4::msg::GpioConfig msg_;
};

class Init_GpioConfig_timestamp
{
public:
  Init_GpioConfig_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GpioConfig_device_id timestamp(::px4::msg::GpioConfig::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_GpioConfig_device_id(msg_);
  }

private:
  ::px4::msg::GpioConfig msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::GpioConfig>()
{
  return px4::msg::builder::Init_GpioConfig_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__GPIO_CONFIG__BUILDER_HPP_
