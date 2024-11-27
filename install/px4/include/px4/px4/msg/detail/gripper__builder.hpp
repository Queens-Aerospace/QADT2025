// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/Gripper.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__GRIPPER__BUILDER_HPP_
#define PX4__MSG__DETAIL__GRIPPER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/gripper__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_Gripper_command
{
public:
  explicit Init_Gripper_command(::px4::msg::Gripper & msg)
  : msg_(msg)
  {}
  ::px4::msg::Gripper command(::px4::msg::Gripper::_command_type arg)
  {
    msg_.command = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::Gripper msg_;
};

class Init_Gripper_timestamp
{
public:
  Init_Gripper_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Gripper_command timestamp(::px4::msg::Gripper::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Gripper_command(msg_);
  }

private:
  ::px4::msg::Gripper msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::Gripper>()
{
  return px4::msg::builder::Init_Gripper_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__GRIPPER__BUILDER_HPP_
