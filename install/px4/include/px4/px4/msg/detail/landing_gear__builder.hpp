// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/LandingGear.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__LANDING_GEAR__BUILDER_HPP_
#define PX4__MSG__DETAIL__LANDING_GEAR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/landing_gear__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_LandingGear_landing_gear
{
public:
  explicit Init_LandingGear_landing_gear(::px4::msg::LandingGear & msg)
  : msg_(msg)
  {}
  ::px4::msg::LandingGear landing_gear(::px4::msg::LandingGear::_landing_gear_type arg)
  {
    msg_.landing_gear = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::LandingGear msg_;
};

class Init_LandingGear_timestamp
{
public:
  Init_LandingGear_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LandingGear_landing_gear timestamp(::px4::msg::LandingGear::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_LandingGear_landing_gear(msg_);
  }

private:
  ::px4::msg::LandingGear msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::LandingGear>()
{
  return px4::msg::builder::Init_LandingGear_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__LANDING_GEAR__BUILDER_HPP_
