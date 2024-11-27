// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/GimbalDeviceSetAttitude.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__BUILDER_HPP_
#define PX4__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/gimbal_device_set_attitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_GimbalDeviceSetAttitude_angular_velocity_z
{
public:
  explicit Init_GimbalDeviceSetAttitude_angular_velocity_z(::px4::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  ::px4::msg::GimbalDeviceSetAttitude angular_velocity_z(::px4::msg::GimbalDeviceSetAttitude::_angular_velocity_z_type arg)
  {
    msg_.angular_velocity_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_angular_velocity_y
{
public:
  explicit Init_GimbalDeviceSetAttitude_angular_velocity_y(::px4::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_angular_velocity_z angular_velocity_y(::px4::msg::GimbalDeviceSetAttitude::_angular_velocity_y_type arg)
  {
    msg_.angular_velocity_y = std::move(arg);
    return Init_GimbalDeviceSetAttitude_angular_velocity_z(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_angular_velocity_x
{
public:
  explicit Init_GimbalDeviceSetAttitude_angular_velocity_x(::px4::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_angular_velocity_y angular_velocity_x(::px4::msg::GimbalDeviceSetAttitude::_angular_velocity_x_type arg)
  {
    msg_.angular_velocity_x = std::move(arg);
    return Init_GimbalDeviceSetAttitude_angular_velocity_y(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_q
{
public:
  explicit Init_GimbalDeviceSetAttitude_q(::px4::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_angular_velocity_x q(::px4::msg::GimbalDeviceSetAttitude::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_GimbalDeviceSetAttitude_angular_velocity_x(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_flags
{
public:
  explicit Init_GimbalDeviceSetAttitude_flags(::px4::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_q flags(::px4::msg::GimbalDeviceSetAttitude::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_GimbalDeviceSetAttitude_q(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_target_component
{
public:
  explicit Init_GimbalDeviceSetAttitude_target_component(::px4::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_flags target_component(::px4::msg::GimbalDeviceSetAttitude::_target_component_type arg)
  {
    msg_.target_component = std::move(arg);
    return Init_GimbalDeviceSetAttitude_flags(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_target_system
{
public:
  explicit Init_GimbalDeviceSetAttitude_target_system(::px4::msg::GimbalDeviceSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalDeviceSetAttitude_target_component target_system(::px4::msg::GimbalDeviceSetAttitude::_target_system_type arg)
  {
    msg_.target_system = std::move(arg);
    return Init_GimbalDeviceSetAttitude_target_component(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

class Init_GimbalDeviceSetAttitude_timestamp
{
public:
  Init_GimbalDeviceSetAttitude_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalDeviceSetAttitude_target_system timestamp(::px4::msg::GimbalDeviceSetAttitude::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_GimbalDeviceSetAttitude_target_system(msg_);
  }

private:
  ::px4::msg::GimbalDeviceSetAttitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::GimbalDeviceSetAttitude>()
{
  return px4::msg::builder::Init_GimbalDeviceSetAttitude_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__GIMBAL_DEVICE_SET_ATTITUDE__BUILDER_HPP_
