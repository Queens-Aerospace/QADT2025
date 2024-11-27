// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/LoggerStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__LOGGER_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__LOGGER_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/logger_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_LoggerStatus_num_messages
{
public:
  explicit Init_LoggerStatus_num_messages(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::LoggerStatus num_messages(::px4::msg::LoggerStatus::_num_messages_type arg)
  {
    msg_.num_messages = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_buffer_size_bytes
{
public:
  explicit Init_LoggerStatus_buffer_size_bytes(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_num_messages buffer_size_bytes(::px4::msg::LoggerStatus::_buffer_size_bytes_type arg)
  {
    msg_.buffer_size_bytes = std::move(arg);
    return Init_LoggerStatus_num_messages(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_buffer_used_bytes
{
public:
  explicit Init_LoggerStatus_buffer_used_bytes(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_buffer_size_bytes buffer_used_bytes(::px4::msg::LoggerStatus::_buffer_used_bytes_type arg)
  {
    msg_.buffer_used_bytes = std::move(arg);
    return Init_LoggerStatus_buffer_size_bytes(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_message_gaps
{
public:
  explicit Init_LoggerStatus_message_gaps(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_buffer_used_bytes message_gaps(::px4::msg::LoggerStatus::_message_gaps_type arg)
  {
    msg_.message_gaps = std::move(arg);
    return Init_LoggerStatus_buffer_used_bytes(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_dropouts
{
public:
  explicit Init_LoggerStatus_dropouts(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_message_gaps dropouts(::px4::msg::LoggerStatus::_dropouts_type arg)
  {
    msg_.dropouts = std::move(arg);
    return Init_LoggerStatus_message_gaps(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_write_rate_kb_s
{
public:
  explicit Init_LoggerStatus_write_rate_kb_s(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_dropouts write_rate_kb_s(::px4::msg::LoggerStatus::_write_rate_kb_s_type arg)
  {
    msg_.write_rate_kb_s = std::move(arg);
    return Init_LoggerStatus_dropouts(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_total_written_kb
{
public:
  explicit Init_LoggerStatus_total_written_kb(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_write_rate_kb_s total_written_kb(::px4::msg::LoggerStatus::_total_written_kb_type arg)
  {
    msg_.total_written_kb = std::move(arg);
    return Init_LoggerStatus_write_rate_kb_s(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_is_logging
{
public:
  explicit Init_LoggerStatus_is_logging(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_total_written_kb is_logging(::px4::msg::LoggerStatus::_is_logging_type arg)
  {
    msg_.is_logging = std::move(arg);
    return Init_LoggerStatus_total_written_kb(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_backend
{
public:
  explicit Init_LoggerStatus_backend(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_is_logging backend(::px4::msg::LoggerStatus::_backend_type arg)
  {
    msg_.backend = std::move(arg);
    return Init_LoggerStatus_is_logging(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_type
{
public:
  explicit Init_LoggerStatus_type(::px4::msg::LoggerStatus & msg)
  : msg_(msg)
  {}
  Init_LoggerStatus_backend type(::px4::msg::LoggerStatus::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_LoggerStatus_backend(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

class Init_LoggerStatus_timestamp
{
public:
  Init_LoggerStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LoggerStatus_type timestamp(::px4::msg::LoggerStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_LoggerStatus_type(msg_);
  }

private:
  ::px4::msg::LoggerStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::LoggerStatus>()
{
  return px4::msg::builder::Init_LoggerStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__LOGGER_STATUS__BUILDER_HPP_
