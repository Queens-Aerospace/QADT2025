// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/OpenDroneIdOperatorId.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__BUILDER_HPP_
#define PX4__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/open_drone_id_operator_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIdOperatorId_operator_id
{
public:
  explicit Init_OpenDroneIdOperatorId_operator_id(::px4::msg::OpenDroneIdOperatorId & msg)
  : msg_(msg)
  {}
  ::px4::msg::OpenDroneIdOperatorId operator_id(::px4::msg::OpenDroneIdOperatorId::_operator_id_type arg)
  {
    msg_.operator_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::OpenDroneIdOperatorId msg_;
};

class Init_OpenDroneIdOperatorId_operator_id_type
{
public:
  explicit Init_OpenDroneIdOperatorId_operator_id_type(::px4::msg::OpenDroneIdOperatorId & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIdOperatorId_operator_id operator_id_type(::px4::msg::OpenDroneIdOperatorId::_operator_id_type_type arg)
  {
    msg_.operator_id_type = std::move(arg);
    return Init_OpenDroneIdOperatorId_operator_id(msg_);
  }

private:
  ::px4::msg::OpenDroneIdOperatorId msg_;
};

class Init_OpenDroneIdOperatorId_id_or_mac
{
public:
  explicit Init_OpenDroneIdOperatorId_id_or_mac(::px4::msg::OpenDroneIdOperatorId & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIdOperatorId_operator_id_type id_or_mac(::px4::msg::OpenDroneIdOperatorId::_id_or_mac_type arg)
  {
    msg_.id_or_mac = std::move(arg);
    return Init_OpenDroneIdOperatorId_operator_id_type(msg_);
  }

private:
  ::px4::msg::OpenDroneIdOperatorId msg_;
};

class Init_OpenDroneIdOperatorId_timestamp
{
public:
  Init_OpenDroneIdOperatorId_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIdOperatorId_id_or_mac timestamp(::px4::msg::OpenDroneIdOperatorId::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_OpenDroneIdOperatorId_id_or_mac(msg_);
  }

private:
  ::px4::msg::OpenDroneIdOperatorId msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::OpenDroneIdOperatorId>()
{
  return px4::msg::builder::Init_OpenDroneIdOperatorId_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__OPEN_DRONE_ID_OPERATOR_ID__BUILDER_HPP_
