// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/MountOrientation.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MOUNT_ORIENTATION__BUILDER_HPP_
#define PX4__MSG__DETAIL__MOUNT_ORIENTATION__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/mount_orientation__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_MountOrientation_attitude_euler_angle
{
public:
  explicit Init_MountOrientation_attitude_euler_angle(::px4::msg::MountOrientation & msg)
  : msg_(msg)
  {}
  ::px4::msg::MountOrientation attitude_euler_angle(::px4::msg::MountOrientation::_attitude_euler_angle_type arg)
  {
    msg_.attitude_euler_angle = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::MountOrientation msg_;
};

class Init_MountOrientation_timestamp
{
public:
  Init_MountOrientation_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MountOrientation_attitude_euler_angle timestamp(::px4::msg::MountOrientation::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_MountOrientation_attitude_euler_angle(msg_);
  }

private:
  ::px4::msg::MountOrientation msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::MountOrientation>()
{
  return px4::msg::builder::Init_MountOrientation_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__MOUNT_ORIENTATION__BUILDER_HPP_
