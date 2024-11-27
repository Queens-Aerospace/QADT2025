// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/OrbitStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ORBIT_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__ORBIT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/orbit_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_OrbitStatus_yaw_behaviour
{
public:
  explicit Init_OrbitStatus_yaw_behaviour(::px4::msg::OrbitStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::OrbitStatus yaw_behaviour(::px4::msg::OrbitStatus::_yaw_behaviour_type arg)
  {
    msg_.yaw_behaviour = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::OrbitStatus msg_;
};

class Init_OrbitStatus_z
{
public:
  explicit Init_OrbitStatus_z(::px4::msg::OrbitStatus & msg)
  : msg_(msg)
  {}
  Init_OrbitStatus_yaw_behaviour z(::px4::msg::OrbitStatus::_z_type arg)
  {
    msg_.z = std::move(arg);
    return Init_OrbitStatus_yaw_behaviour(msg_);
  }

private:
  ::px4::msg::OrbitStatus msg_;
};

class Init_OrbitStatus_y
{
public:
  explicit Init_OrbitStatus_y(::px4::msg::OrbitStatus & msg)
  : msg_(msg)
  {}
  Init_OrbitStatus_z y(::px4::msg::OrbitStatus::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_OrbitStatus_z(msg_);
  }

private:
  ::px4::msg::OrbitStatus msg_;
};

class Init_OrbitStatus_x
{
public:
  explicit Init_OrbitStatus_x(::px4::msg::OrbitStatus & msg)
  : msg_(msg)
  {}
  Init_OrbitStatus_y x(::px4::msg::OrbitStatus::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_OrbitStatus_y(msg_);
  }

private:
  ::px4::msg::OrbitStatus msg_;
};

class Init_OrbitStatus_frame
{
public:
  explicit Init_OrbitStatus_frame(::px4::msg::OrbitStatus & msg)
  : msg_(msg)
  {}
  Init_OrbitStatus_x frame(::px4::msg::OrbitStatus::_frame_type arg)
  {
    msg_.frame = std::move(arg);
    return Init_OrbitStatus_x(msg_);
  }

private:
  ::px4::msg::OrbitStatus msg_;
};

class Init_OrbitStatus_radius
{
public:
  explicit Init_OrbitStatus_radius(::px4::msg::OrbitStatus & msg)
  : msg_(msg)
  {}
  Init_OrbitStatus_frame radius(::px4::msg::OrbitStatus::_radius_type arg)
  {
    msg_.radius = std::move(arg);
    return Init_OrbitStatus_frame(msg_);
  }

private:
  ::px4::msg::OrbitStatus msg_;
};

class Init_OrbitStatus_timestamp
{
public:
  Init_OrbitStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrbitStatus_radius timestamp(::px4::msg::OrbitStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_OrbitStatus_radius(msg_);
  }

private:
  ::px4::msg::OrbitStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::OrbitStatus>()
{
  return px4::msg::builder::Init_OrbitStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ORBIT_STATUS__BUILDER_HPP_
