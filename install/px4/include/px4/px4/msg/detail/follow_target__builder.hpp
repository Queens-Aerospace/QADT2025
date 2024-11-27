// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/FollowTarget.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__FOLLOW_TARGET__BUILDER_HPP_
#define PX4__MSG__DETAIL__FOLLOW_TARGET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/follow_target__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_FollowTarget_est_cap
{
public:
  explicit Init_FollowTarget_est_cap(::px4::msg::FollowTarget & msg)
  : msg_(msg)
  {}
  ::px4::msg::FollowTarget est_cap(::px4::msg::FollowTarget::_est_cap_type arg)
  {
    msg_.est_cap = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

class Init_FollowTarget_vz
{
public:
  explicit Init_FollowTarget_vz(::px4::msg::FollowTarget & msg)
  : msg_(msg)
  {}
  Init_FollowTarget_est_cap vz(::px4::msg::FollowTarget::_vz_type arg)
  {
    msg_.vz = std::move(arg);
    return Init_FollowTarget_est_cap(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

class Init_FollowTarget_vx
{
public:
  explicit Init_FollowTarget_vx(::px4::msg::FollowTarget & msg)
  : msg_(msg)
  {}
  Init_FollowTarget_vz vx(::px4::msg::FollowTarget::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_FollowTarget_vz(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

class Init_FollowTarget_vy
{
public:
  explicit Init_FollowTarget_vy(::px4::msg::FollowTarget & msg)
  : msg_(msg)
  {}
  Init_FollowTarget_vx vy(::px4::msg::FollowTarget::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_FollowTarget_vx(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

class Init_FollowTarget_alt
{
public:
  explicit Init_FollowTarget_alt(::px4::msg::FollowTarget & msg)
  : msg_(msg)
  {}
  Init_FollowTarget_vy alt(::px4::msg::FollowTarget::_alt_type arg)
  {
    msg_.alt = std::move(arg);
    return Init_FollowTarget_vy(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

class Init_FollowTarget_lon
{
public:
  explicit Init_FollowTarget_lon(::px4::msg::FollowTarget & msg)
  : msg_(msg)
  {}
  Init_FollowTarget_alt lon(::px4::msg::FollowTarget::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_FollowTarget_alt(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

class Init_FollowTarget_lat
{
public:
  explicit Init_FollowTarget_lat(::px4::msg::FollowTarget & msg)
  : msg_(msg)
  {}
  Init_FollowTarget_lon lat(::px4::msg::FollowTarget::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_FollowTarget_lon(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

class Init_FollowTarget_timestamp
{
public:
  Init_FollowTarget_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowTarget_lat timestamp(::px4::msg::FollowTarget::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_FollowTarget_lat(msg_);
  }

private:
  ::px4::msg::FollowTarget msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::FollowTarget>()
{
  return px4::msg::builder::Init_FollowTarget_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__FOLLOW_TARGET__BUILDER_HPP_
