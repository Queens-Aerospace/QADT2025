// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/OpenDroneIdSelfId.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__BUILDER_HPP_
#define PX4__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/open_drone_id_self_id__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_OpenDroneIdSelfId_description
{
public:
  explicit Init_OpenDroneIdSelfId_description(::px4::msg::OpenDroneIdSelfId & msg)
  : msg_(msg)
  {}
  ::px4::msg::OpenDroneIdSelfId description(::px4::msg::OpenDroneIdSelfId::_description_type arg)
  {
    msg_.description = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::OpenDroneIdSelfId msg_;
};

class Init_OpenDroneIdSelfId_description_type
{
public:
  explicit Init_OpenDroneIdSelfId_description_type(::px4::msg::OpenDroneIdSelfId & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIdSelfId_description description_type(::px4::msg::OpenDroneIdSelfId::_description_type_type arg)
  {
    msg_.description_type = std::move(arg);
    return Init_OpenDroneIdSelfId_description(msg_);
  }

private:
  ::px4::msg::OpenDroneIdSelfId msg_;
};

class Init_OpenDroneIdSelfId_id_or_mac
{
public:
  explicit Init_OpenDroneIdSelfId_id_or_mac(::px4::msg::OpenDroneIdSelfId & msg)
  : msg_(msg)
  {}
  Init_OpenDroneIdSelfId_description_type id_or_mac(::px4::msg::OpenDroneIdSelfId::_id_or_mac_type arg)
  {
    msg_.id_or_mac = std::move(arg);
    return Init_OpenDroneIdSelfId_description_type(msg_);
  }

private:
  ::px4::msg::OpenDroneIdSelfId msg_;
};

class Init_OpenDroneIdSelfId_timestamp
{
public:
  Init_OpenDroneIdSelfId_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OpenDroneIdSelfId_id_or_mac timestamp(::px4::msg::OpenDroneIdSelfId::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_OpenDroneIdSelfId_id_or_mac(msg_);
  }

private:
  ::px4::msg::OpenDroneIdSelfId msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::OpenDroneIdSelfId>()
{
  return px4::msg::builder::Init_OpenDroneIdSelfId_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__OPEN_DRONE_ID_SELF_ID__BUILDER_HPP_
