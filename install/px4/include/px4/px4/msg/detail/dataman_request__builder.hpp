// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/DatamanRequest.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__DATAMAN_REQUEST__BUILDER_HPP_
#define PX4__MSG__DETAIL__DATAMAN_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/dataman_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_DatamanRequest_data_length
{
public:
  explicit Init_DatamanRequest_data_length(::px4::msg::DatamanRequest & msg)
  : msg_(msg)
  {}
  ::px4::msg::DatamanRequest data_length(::px4::msg::DatamanRequest::_data_length_type arg)
  {
    msg_.data_length = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::DatamanRequest msg_;
};

class Init_DatamanRequest_data
{
public:
  explicit Init_DatamanRequest_data(::px4::msg::DatamanRequest & msg)
  : msg_(msg)
  {}
  Init_DatamanRequest_data_length data(::px4::msg::DatamanRequest::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_DatamanRequest_data_length(msg_);
  }

private:
  ::px4::msg::DatamanRequest msg_;
};

class Init_DatamanRequest_index
{
public:
  explicit Init_DatamanRequest_index(::px4::msg::DatamanRequest & msg)
  : msg_(msg)
  {}
  Init_DatamanRequest_data index(::px4::msg::DatamanRequest::_index_type arg)
  {
    msg_.index = std::move(arg);
    return Init_DatamanRequest_data(msg_);
  }

private:
  ::px4::msg::DatamanRequest msg_;
};

class Init_DatamanRequest_item
{
public:
  explicit Init_DatamanRequest_item(::px4::msg::DatamanRequest & msg)
  : msg_(msg)
  {}
  Init_DatamanRequest_index item(::px4::msg::DatamanRequest::_item_type arg)
  {
    msg_.item = std::move(arg);
    return Init_DatamanRequest_index(msg_);
  }

private:
  ::px4::msg::DatamanRequest msg_;
};

class Init_DatamanRequest_request_type
{
public:
  explicit Init_DatamanRequest_request_type(::px4::msg::DatamanRequest & msg)
  : msg_(msg)
  {}
  Init_DatamanRequest_item request_type(::px4::msg::DatamanRequest::_request_type_type arg)
  {
    msg_.request_type = std::move(arg);
    return Init_DatamanRequest_item(msg_);
  }

private:
  ::px4::msg::DatamanRequest msg_;
};

class Init_DatamanRequest_client_id
{
public:
  explicit Init_DatamanRequest_client_id(::px4::msg::DatamanRequest & msg)
  : msg_(msg)
  {}
  Init_DatamanRequest_request_type client_id(::px4::msg::DatamanRequest::_client_id_type arg)
  {
    msg_.client_id = std::move(arg);
    return Init_DatamanRequest_request_type(msg_);
  }

private:
  ::px4::msg::DatamanRequest msg_;
};

class Init_DatamanRequest_timestamp
{
public:
  Init_DatamanRequest_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_DatamanRequest_client_id timestamp(::px4::msg::DatamanRequest::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_DatamanRequest_client_id(msg_);
  }

private:
  ::px4::msg::DatamanRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::DatamanRequest>()
{
  return px4::msg::builder::Init_DatamanRequest_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__DATAMAN_REQUEST__BUILDER_HPP_
