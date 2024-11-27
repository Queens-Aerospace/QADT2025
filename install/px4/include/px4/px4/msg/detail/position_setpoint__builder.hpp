// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/PositionSetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__POSITION_SETPOINT__BUILDER_HPP_
#define PX4__MSG__DETAIL__POSITION_SETPOINT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/position_setpoint__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_PositionSetpoint_cruising_throttle
{
public:
  explicit Init_PositionSetpoint_cruising_throttle(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  ::px4::msg::PositionSetpoint cruising_throttle(::px4::msg::PositionSetpoint::_cruising_throttle_type arg)
  {
    msg_.cruising_throttle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_gliding_enabled
{
public:
  explicit Init_PositionSetpoint_gliding_enabled(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_cruising_throttle gliding_enabled(::px4::msg::PositionSetpoint::_gliding_enabled_type arg)
  {
    msg_.gliding_enabled = std::move(arg);
    return Init_PositionSetpoint_cruising_throttle(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_cruising_speed
{
public:
  explicit Init_PositionSetpoint_cruising_speed(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_gliding_enabled cruising_speed(::px4::msg::PositionSetpoint::_cruising_speed_type arg)
  {
    msg_.cruising_speed = std::move(arg);
    return Init_PositionSetpoint_gliding_enabled(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_alt_acceptance_radius
{
public:
  explicit Init_PositionSetpoint_alt_acceptance_radius(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_cruising_speed alt_acceptance_radius(::px4::msg::PositionSetpoint::_alt_acceptance_radius_type arg)
  {
    msg_.alt_acceptance_radius = std::move(arg);
    return Init_PositionSetpoint_cruising_speed(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_acceptance_radius
{
public:
  explicit Init_PositionSetpoint_acceptance_radius(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_alt_acceptance_radius acceptance_radius(::px4::msg::PositionSetpoint::_acceptance_radius_type arg)
  {
    msg_.acceptance_radius = std::move(arg);
    return Init_PositionSetpoint_alt_acceptance_radius(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_loiter_pattern
{
public:
  explicit Init_PositionSetpoint_loiter_pattern(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_acceptance_radius loiter_pattern(::px4::msg::PositionSetpoint::_loiter_pattern_type arg)
  {
    msg_.loiter_pattern = std::move(arg);
    return Init_PositionSetpoint_acceptance_radius(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_loiter_orientation
{
public:
  explicit Init_PositionSetpoint_loiter_orientation(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_loiter_pattern loiter_orientation(::px4::msg::PositionSetpoint::_loiter_orientation_type arg)
  {
    msg_.loiter_orientation = std::move(arg);
    return Init_PositionSetpoint_loiter_pattern(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_loiter_direction_counter_clockwise
{
public:
  explicit Init_PositionSetpoint_loiter_direction_counter_clockwise(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_loiter_orientation loiter_direction_counter_clockwise(::px4::msg::PositionSetpoint::_loiter_direction_counter_clockwise_type arg)
  {
    msg_.loiter_direction_counter_clockwise = std::move(arg);
    return Init_PositionSetpoint_loiter_orientation(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_loiter_minor_radius
{
public:
  explicit Init_PositionSetpoint_loiter_minor_radius(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_loiter_direction_counter_clockwise loiter_minor_radius(::px4::msg::PositionSetpoint::_loiter_minor_radius_type arg)
  {
    msg_.loiter_minor_radius = std::move(arg);
    return Init_PositionSetpoint_loiter_direction_counter_clockwise(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_loiter_radius
{
public:
  explicit Init_PositionSetpoint_loiter_radius(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_loiter_minor_radius loiter_radius(::px4::msg::PositionSetpoint::_loiter_radius_type arg)
  {
    msg_.loiter_radius = std::move(arg);
    return Init_PositionSetpoint_loiter_minor_radius(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_yaw
{
public:
  explicit Init_PositionSetpoint_yaw(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_loiter_radius yaw(::px4::msg::PositionSetpoint::_yaw_type arg)
  {
    msg_.yaw = std::move(arg);
    return Init_PositionSetpoint_loiter_radius(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_alt
{
public:
  explicit Init_PositionSetpoint_alt(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_yaw alt(::px4::msg::PositionSetpoint::_alt_type arg)
  {
    msg_.alt = std::move(arg);
    return Init_PositionSetpoint_yaw(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_lon
{
public:
  explicit Init_PositionSetpoint_lon(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_alt lon(::px4::msg::PositionSetpoint::_lon_type arg)
  {
    msg_.lon = std::move(arg);
    return Init_PositionSetpoint_alt(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_lat
{
public:
  explicit Init_PositionSetpoint_lat(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_lon lat(::px4::msg::PositionSetpoint::_lat_type arg)
  {
    msg_.lat = std::move(arg);
    return Init_PositionSetpoint_lon(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_vz
{
public:
  explicit Init_PositionSetpoint_vz(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_lat vz(::px4::msg::PositionSetpoint::_vz_type arg)
  {
    msg_.vz = std::move(arg);
    return Init_PositionSetpoint_lat(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_vy
{
public:
  explicit Init_PositionSetpoint_vy(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_vz vy(::px4::msg::PositionSetpoint::_vy_type arg)
  {
    msg_.vy = std::move(arg);
    return Init_PositionSetpoint_vz(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_vx
{
public:
  explicit Init_PositionSetpoint_vx(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_vy vx(::px4::msg::PositionSetpoint::_vx_type arg)
  {
    msg_.vx = std::move(arg);
    return Init_PositionSetpoint_vy(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_type
{
public:
  explicit Init_PositionSetpoint_type(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_vx type(::px4::msg::PositionSetpoint::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_PositionSetpoint_vx(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_valid
{
public:
  explicit Init_PositionSetpoint_valid(::px4::msg::PositionSetpoint & msg)
  : msg_(msg)
  {}
  Init_PositionSetpoint_type valid(::px4::msg::PositionSetpoint::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_PositionSetpoint_type(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

class Init_PositionSetpoint_timestamp
{
public:
  Init_PositionSetpoint_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PositionSetpoint_valid timestamp(::px4::msg::PositionSetpoint::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_PositionSetpoint_valid(msg_);
  }

private:
  ::px4::msg::PositionSetpoint msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::PositionSetpoint>()
{
  return px4::msg::builder::Init_PositionSetpoint_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__POSITION_SETPOINT__BUILDER_HPP_
