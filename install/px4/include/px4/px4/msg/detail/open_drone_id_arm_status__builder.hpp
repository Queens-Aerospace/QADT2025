// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/OpenDroneIdArmStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__OPEN_DRONE_ID_ARM_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__OPEN_DRONE_ID_ARM_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/open_drone_id_arm_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIdArmStatus_error
{
public:
  explicit Init_OpenDroneIdArmStatus_error(::px4::msg::OpenDroneIdArmStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::OpenDroneIdArmStatus error(::px4::msg::OpenDroneIdArmStatus::_error_type arg)
  {
    msg_.error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::OpenDroneIdArmStatus msg_;
};

class Init_OpenDroneIdArmStatus_status
{
public:
  explicit Init_OpenDroneIdArmStatus_status(::px4::msg::OpenDroneIdArmStatus & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIdArmStatus_error status(::px4::msg::OpenDroneIdArmStatus::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_OpenDroneIdArmStatus_error(msg_);
  }

private:
  ::px4::msg::OpenDroneIdArmStatus msg_;
};

class Init_OpenDroneIdArmStatus_timestamp
{
public:
  Init_OpenDroneIdArmStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIdArmStatus_status timestamp(::px4::msg::OpenDroneIdArmStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_OpenDroneIdArmStatus_status(msg_);
  }

private:
  ::px4::msg::OpenDroneIdArmStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::OpenDroneIdArmStatus>()
{
  return px4::msg::builder::Init_OpenDroneIdArmStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__OPEN_DRONE_ID_ARM_STATUS__BUILDER_HPP_
