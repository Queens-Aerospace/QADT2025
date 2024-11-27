// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/ParamVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/param_vec__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_ParamVec_params
{
public:
  explicit Init_ParamVec_params(::ros_gz_interfaces::msg::ParamVec & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::ParamVec params(::ros_gz_interfaces::msg::ParamVec::_params_type arg)
  {
    msg_.params = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::ParamVec msg_;
};

class Init_ParamVec_header
{
public:
  Init_ParamVec_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ParamVec_params header(::ros_gz_interfaces::msg::ParamVec::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ParamVec_params(msg_);
  }

private:
  ::ros_gz_interfaces::msg::ParamVec msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::ParamVec>()
{
  return ros_gz_interfaces::msg::builder::Init_ParamVec_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__BUILDER_HPP_
