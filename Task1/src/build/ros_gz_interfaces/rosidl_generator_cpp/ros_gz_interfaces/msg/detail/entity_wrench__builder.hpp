// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/EntityWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/entity_wrench__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_EntityWrench_wrench
{
public:
  explicit Init_EntityWrench_wrench(::ros_gz_interfaces::msg::EntityWrench & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::EntityWrench wrench(::ros_gz_interfaces::msg::EntityWrench::_wrench_type arg)
  {
    msg_.wrench = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityWrench msg_;
};

class Init_EntityWrench_entity
{
public:
  explicit Init_EntityWrench_entity(::ros_gz_interfaces::msg::EntityWrench & msg)
  : msg_(msg)
  {}
  Init_EntityWrench_wrench entity(::ros_gz_interfaces::msg::EntityWrench::_entity_type arg)
  {
    msg_.entity = std::move(arg);
    return Init_EntityWrench_wrench(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityWrench msg_;
};

class Init_EntityWrench_header
{
public:
  Init_EntityWrench_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EntityWrench_entity header(::ros_gz_interfaces::msg::EntityWrench::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_EntityWrench_entity(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityWrench msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::EntityWrench>()
{
  return ros_gz_interfaces::msg::builder::Init_EntityWrench_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__BUILDER_HPP_
