// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/MessageFormatRequest.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MESSAGE_FORMAT_REQUEST__BUILDER_HPP_
#define PX4__MSG__DETAIL__MESSAGE_FORMAT_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/message_format_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_MessageFormatRequest_topic_name
{
public:
  explicit Init_MessageFormatRequest_topic_name(::px4::msg::MessageFormatRequest & msg)
  : msg_(msg)
  {}
  ::px4::msg::MessageFormatRequest topic_name(::px4::msg::MessageFormatRequest::_topic_name_type arg)
  {
    msg_.topic_name = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::MessageFormatRequest msg_;
};

class Init_MessageFormatRequest_protocol_version
{
public:
  explicit Init_MessageFormatRequest_protocol_version(::px4::msg::MessageFormatRequest & msg)
  : msg_(msg)
  {}
  Init_MessageFormatRequest_topic_name protocol_version(::px4::msg::MessageFormatRequest::_protocol_version_type arg)
  {
    msg_.protocol_version = std::move(arg);
    return Init_MessageFormatRequest_topic_name(msg_);
  }

private:
  ::px4::msg::MessageFormatRequest msg_;
};

class Init_MessageFormatRequest_timestamp
{
public:
  Init_MessageFormatRequest_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MessageFormatRequest_protocol_version timestamp(::px4::msg::MessageFormatRequest::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_MessageFormatRequest_protocol_version(msg_);
  }

private:
  ::px4::msg::MessageFormatRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::MessageFormatRequest>()
{
  return px4::msg::builder::Init_MessageFormatRequest_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__MESSAGE_FORMAT_REQUEST__BUILDER_HPP_
