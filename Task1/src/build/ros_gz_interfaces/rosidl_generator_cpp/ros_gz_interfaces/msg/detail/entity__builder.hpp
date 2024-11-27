// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/Entity.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/entity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Entity_type
{
public:
  explicit Init_Entity_type(::ros_gz_interfaces::msg::Entity & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::Entity type(::ros_gz_interfaces::msg::Entity::_type_type arg)
  {
    msg_.type = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Entity msg_;
};

class Init_Entity_name
{
public:
  explicit Init_Entity_name(::ros_gz_interfaces::msg::Entity & msg)
  : msg_(msg)
  {}
  Init_Entity_type name(::ros_gz_interfaces::msg::Entity::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Entity_type(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Entity msg_;
};

class Init_Entity_id
{
public:
  Init_Entity_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Entity_name id(::ros_gz_interfaces::msg::Entity::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Entity_name(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Entity msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::Entity>()
{
  return ros_gz_interfaces::msg::builder::Init_Entity_id();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__BUILDER_HPP_
