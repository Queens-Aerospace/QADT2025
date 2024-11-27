// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/WheelEncoders.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__WHEEL_ENCODERS__BUILDER_HPP_
#define PX4__MSG__DETAIL__WHEEL_ENCODERS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/wheel_encoders__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_WheelEncoders_wheel_angle
{
public:
  explicit Init_WheelEncoders_wheel_angle(::px4::msg::WheelEncoders & msg)
  : msg_(msg)
  {}
  ::px4::msg::WheelEncoders wheel_angle(::px4::msg::WheelEncoders::_wheel_angle_type arg)
  {
    msg_.wheel_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::WheelEncoders msg_;
};

class Init_WheelEncoders_wheel_speed
{
public:
  explicit Init_WheelEncoders_wheel_speed(::px4::msg::WheelEncoders & msg)
  : msg_(msg)
  {}
  Init_WheelEncoders_wheel_angle wheel_speed(::px4::msg::WheelEncoders::_wheel_speed_type arg)
  {
    msg_.wheel_speed = std::move(arg);
    return Init_WheelEncoders_wheel_angle(msg_);
  }

private:
  ::px4::msg::WheelEncoders msg_;
};

class Init_WheelEncoders_timestamp
{
public:
  Init_WheelEncoders_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WheelEncoders_wheel_speed timestamp(::px4::msg::WheelEncoders::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_WheelEncoders_wheel_speed(msg_);
  }

private:
  ::px4::msg::WheelEncoders msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::WheelEncoders>()
{
  return px4::msg::builder::Init_WheelEncoders_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__WHEEL_ENCODERS__BUILDER_HPP_
