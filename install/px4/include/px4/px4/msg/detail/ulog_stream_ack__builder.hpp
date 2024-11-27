// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/UlogStreamAck.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ULOG_STREAM_ACK__BUILDER_HPP_
#define PX4__MSG__DETAIL__ULOG_STREAM_ACK__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/ulog_stream_ack__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_UlogStreamAck_msg_sequence
{
public:
  explicit Init_UlogStreamAck_msg_sequence(::px4::msg::UlogStreamAck & msg)
  : msg_(msg)
  {}
  ::px4::msg::UlogStreamAck msg_sequence(::px4::msg::UlogStreamAck::_msg_sequence_type arg)
  {
    msg_.msg_sequence = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::UlogStreamAck msg_;
};

class Init_UlogStreamAck_timestamp
{
public:
  Init_UlogStreamAck_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_UlogStreamAck_msg_sequence timestamp(::px4::msg::UlogStreamAck::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_UlogStreamAck_msg_sequence(msg_);
  }

private:
  ::px4::msg::UlogStreamAck msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::UlogStreamAck>()
{
  return px4::msg::builder::Init_UlogStreamAck_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ULOG_STREAM_ACK__BUILDER_HPP_
