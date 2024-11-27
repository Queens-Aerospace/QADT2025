// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/GimbalManagerSetAttitude.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__GIMBAL_MANAGER_SET_ATTITUDE__BUILDER_HPP_
#define PX4__MSG__DETAIL__GIMBAL_MANAGER_SET_ATTITUDE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/gimbal_manager_set_attitude__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_GimbalManagerSetAttitude_angular_velocity_z
{
public:
  explicit Init_GimbalManagerSetAttitude_angular_velocity_z(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  ::px4::msg::GimbalManagerSetAttitude angular_velocity_z(::px4::msg::GimbalManagerSetAttitude::_angular_velocity_z_type arg)
  {
    msg_.angular_velocity_z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_angular_velocity_y
{
public:
  explicit Init_GimbalManagerSetAttitude_angular_velocity_y(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_angular_velocity_z angular_velocity_y(::px4::msg::GimbalManagerSetAttitude::_angular_velocity_y_type arg)
  {
    msg_.angular_velocity_y = std::move(arg);
    return Init_GimbalManagerSetAttitude_angular_velocity_z(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_angular_velocity_x
{
public:
  explicit Init_GimbalManagerSetAttitude_angular_velocity_x(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_angular_velocity_y angular_velocity_x(::px4::msg::GimbalManagerSetAttitude::_angular_velocity_x_type arg)
  {
    msg_.angular_velocity_x = std::move(arg);
    return Init_GimbalManagerSetAttitude_angular_velocity_y(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_q
{
public:
  explicit Init_GimbalManagerSetAttitude_q(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_angular_velocity_x q(::px4::msg::GimbalManagerSetAttitude::_q_type arg)
  {
    msg_.q = std::move(arg);
    return Init_GimbalManagerSetAttitude_angular_velocity_x(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_gimbal_device_id
{
public:
  explicit Init_GimbalManagerSetAttitude_gimbal_device_id(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_q gimbal_device_id(::px4::msg::GimbalManagerSetAttitude::_gimbal_device_id_type arg)
  {
    msg_.gimbal_device_id = std::move(arg);
    return Init_GimbalManagerSetAttitude_q(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_flags
{
public:
  explicit Init_GimbalManagerSetAttitude_flags(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_gimbal_device_id flags(::px4::msg::GimbalManagerSetAttitude::_flags_type arg)
  {
    msg_.flags = std::move(arg);
    return Init_GimbalManagerSetAttitude_gimbal_device_id(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_target_component
{
public:
  explicit Init_GimbalManagerSetAttitude_target_component(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_flags target_component(::px4::msg::GimbalManagerSetAttitude::_target_component_type arg)
  {
    msg_.target_component = std::move(arg);
    return Init_GimbalManagerSetAttitude_flags(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_target_system
{
public:
  explicit Init_GimbalManagerSetAttitude_target_system(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_target_component target_system(::px4::msg::GimbalManagerSetAttitude::_target_system_type arg)
  {
    msg_.target_system = std::move(arg);
    return Init_GimbalManagerSetAttitude_target_component(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_origin_compid
{
public:
  explicit Init_GimbalManagerSetAttitude_origin_compid(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_target_system origin_compid(::px4::msg::GimbalManagerSetAttitude::_origin_compid_type arg)
  {
    msg_.origin_compid = std::move(arg);
    return Init_GimbalManagerSetAttitude_target_system(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_origin_sysid
{
public:
  explicit Init_GimbalManagerSetAttitude_origin_sysid(::px4::msg::GimbalManagerSetAttitude & msg)
  : msg_(msg)
  {}
  Init_GimbalManagerSetAttitude_origin_compid origin_sysid(::px4::msg::GimbalManagerSetAttitude::_origin_sysid_type arg)
  {
    msg_.origin_sysid = std::move(arg);
    return Init_GimbalManagerSetAttitude_origin_compid(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

class Init_GimbalManagerSetAttitude_timestamp
{
public:
  Init_GimbalManagerSetAttitude_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_GimbalManagerSetAttitude_origin_sysid timestamp(::px4::msg::GimbalManagerSetAttitude::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_GimbalManagerSetAttitude_origin_sysid(msg_);
  }

private:
  ::px4::msg::GimbalManagerSetAttitude msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::GimbalManagerSetAttitude>()
{
  return px4::msg::builder::Init_GimbalManagerSetAttitude_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__GIMBAL_MANAGER_SET_ATTITUDE__BUILDER_HPP_
