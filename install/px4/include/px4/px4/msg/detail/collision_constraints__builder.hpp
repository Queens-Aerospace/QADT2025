// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/CollisionConstraints.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__COLLISION_CONSTRAINTS__BUILDER_HPP_
#define PX4__MSG__DETAIL__COLLISION_CONSTRAINTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/collision_constraints__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_CollisionConstraints_adapted_setpoint
{
public:
  explicit Init_CollisionConstraints_adapted_setpoint(::px4::msg::CollisionConstraints & msg)
  : msg_(msg)
  {}
  ::px4::msg::CollisionConstraints adapted_setpoint(::px4::msg::CollisionConstraints::_adapted_setpoint_type arg)
  {
    msg_.adapted_setpoint = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::CollisionConstraints msg_;
};

class Init_CollisionConstraints_original_setpoint
{
public:
  explicit Init_CollisionConstraints_original_setpoint(::px4::msg::CollisionConstraints & msg)
  : msg_(msg)
  {}
  Init_CollisionConstraints_adapted_setpoint original_setpoint(::px4::msg::CollisionConstraints::_original_setpoint_type arg)
  {
    msg_.original_setpoint = std::move(arg);
    return Init_CollisionConstraints_adapted_setpoint(msg_);
  }

private:
  ::px4::msg::CollisionConstraints msg_;
};

class Init_CollisionConstraints_timestamp
{
public:
  Init_CollisionConstraints_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CollisionConstraints_original_setpoint timestamp(::px4::msg::CollisionConstraints::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_CollisionConstraints_original_setpoint(msg_);
  }

private:
  ::px4::msg::CollisionConstraints msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::CollisionConstraints>()
{
  return px4::msg::builder::Init_CollisionConstraints_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__COLLISION_CONSTRAINTS__BUILDER_HPP_
