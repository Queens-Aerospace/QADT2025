// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/VehicleConstraints.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__VEHICLE_CONSTRAINTS__BUILDER_HPP_
#define PX4__MSG__DETAIL__VEHICLE_CONSTRAINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/vehicle_constraints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_VehicleConstraints_want_takeoff
{
public:
  explicit Init_VehicleConstraints_want_takeoff(::px4::msg::VehicleConstraints & msg)
  : msg_(msg)
  {}
  ::px4::msg::VehicleConstraints want_takeoff(::px4::msg::VehicleConstraints::_want_takeoff_type arg)
  {
    msg_.want_takeoff = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::VehicleConstraints msg_;
};

class Init_VehicleConstraints_speed_down
{
public:
  explicit Init_VehicleConstraints_speed_down(::px4::msg::VehicleConstraints & msg)
  : msg_(msg)
  {}
  Init_VehicleConstraints_want_takeoff speed_down(::px4::msg::VehicleConstraints::_speed_down_type arg)
  {
    msg_.speed_down = std::move(arg);
    return Init_VehicleConstraints_want_takeoff(msg_);
  }

private:
  ::px4::msg::VehicleConstraints msg_;
};

class Init_VehicleConstraints_speed_up
{
public:
  explicit Init_VehicleConstraints_speed_up(::px4::msg::VehicleConstraints & msg)
  : msg_(msg)
  {}
  Init_VehicleConstraints_speed_down speed_up(::px4::msg::VehicleConstraints::_speed_up_type arg)
  {
    msg_.speed_up = std::move(arg);
    return Init_VehicleConstraints_speed_down(msg_);
  }

private:
  ::px4::msg::VehicleConstraints msg_;
};

class Init_VehicleConstraints_timestamp
{
public:
  Init_VehicleConstraints_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleConstraints_speed_up timestamp(::px4::msg::VehicleConstraints::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_VehicleConstraints_speed_up(msg_);
  }

private:
  ::px4::msg::VehicleConstraints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::VehicleConstraints>()
{
  return px4::msg::builder::Init_VehicleConstraints_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__VEHICLE_CONSTRAINTS__BUILDER_HPP_
