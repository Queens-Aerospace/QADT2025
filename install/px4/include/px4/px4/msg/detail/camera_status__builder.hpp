// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/CameraStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__CAMERA_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__CAMERA_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/camera_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_CameraStatus_active_comp_id
{
public:
  explicit Init_CameraStatus_active_comp_id(::px4::msg::CameraStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::CameraStatus active_comp_id(::px4::msg::CameraStatus::_active_comp_id_type arg)
  {
    msg_.active_comp_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::CameraStatus msg_;
};

class Init_CameraStatus_active_sys_id
{
public:
  explicit Init_CameraStatus_active_sys_id(::px4::msg::CameraStatus & msg)
  : msg_(msg)
  {}
  Init_CameraStatus_active_comp_id active_sys_id(::px4::msg::CameraStatus::_active_sys_id_type arg)
  {
    msg_.active_sys_id = std::move(arg);
    return Init_CameraStatus_active_comp_id(msg_);
  }

private:
  ::px4::msg::CameraStatus msg_;
};

class Init_CameraStatus_timestamp
{
public:
  Init_CameraStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_CameraStatus_active_sys_id timestamp(::px4::msg::CameraStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_CameraStatus_active_sys_id(msg_);
  }

private:
  ::px4::msg::CameraStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::CameraStatus>()
{
  return px4::msg::builder::Init_CameraStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__CAMERA_STATUS__BUILDER_HPP_
