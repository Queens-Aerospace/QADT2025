// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/WorldReset.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/world_reset__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_WorldReset_model_only
{
public:
  explicit Init_WorldReset_model_only(::ros_gz_interfaces::msg::WorldReset & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::WorldReset model_only(::ros_gz_interfaces::msg::WorldReset::_model_only_type arg)
  {
    msg_.model_only = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldReset msg_;
};

class Init_WorldReset_time_only
{
public:
  explicit Init_WorldReset_time_only(::ros_gz_interfaces::msg::WorldReset & msg)
  : msg_(msg)
  {}
  Init_WorldReset_model_only time_only(::ros_gz_interfaces::msg::WorldReset::_time_only_type arg)
  {
    msg_.time_only = std::move(arg);
    return Init_WorldReset_model_only(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldReset msg_;
};

class Init_WorldReset_all
{
public:
  Init_WorldReset_all()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorldReset_time_only all(::ros_gz_interfaces::msg::WorldReset::_all_type arg)
  {
    msg_.all = std::move(arg);
    return Init_WorldReset_time_only(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldReset msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::WorldReset>()
{
  return ros_gz_interfaces::msg::builder::Init_WorldReset_all();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_RESET__BUILDER_HPP_
