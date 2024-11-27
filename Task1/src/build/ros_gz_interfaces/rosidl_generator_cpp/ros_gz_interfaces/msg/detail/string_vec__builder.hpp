// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/StringVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__STRING_VEC__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__STRING_VEC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/string_vec__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_StringVec_data
{
public:
  explicit Init_StringVec_data(::ros_gz_interfaces::msg::StringVec & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::StringVec data(::ros_gz_interfaces::msg::StringVec::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::StringVec msg_;
};

class Init_StringVec_header
{
public:
  Init_StringVec_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_StringVec_data header(::ros_gz_interfaces::msg::StringVec::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_StringVec_data(msg_);
  }

private:
  ::ros_gz_interfaces::msg::StringVec msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::StringVec>()
{
  return ros_gz_interfaces::msg::builder::Init_StringVec_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__STRING_VEC__BUILDER_HPP_
