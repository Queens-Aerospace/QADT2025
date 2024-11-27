// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/entity_factory__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_EntityFactory_relative_to
{
public:
  explicit Init_EntityFactory_relative_to(::ros_gz_interfaces::msg::EntityFactory & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::EntityFactory relative_to(::ros_gz_interfaces::msg::EntityFactory::_relative_to_type arg)
  {
    msg_.relative_to = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityFactory msg_;
};

class Init_EntityFactory_pose
{
public:
  explicit Init_EntityFactory_pose(::ros_gz_interfaces::msg::EntityFactory & msg)
  : msg_(msg)
  {}
  Init_EntityFactory_relative_to pose(::ros_gz_interfaces::msg::EntityFactory::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_EntityFactory_relative_to(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityFactory msg_;
};

class Init_EntityFactory_clone_name
{
public:
  explicit Init_EntityFactory_clone_name(::ros_gz_interfaces::msg::EntityFactory & msg)
  : msg_(msg)
  {}
  Init_EntityFactory_pose clone_name(::ros_gz_interfaces::msg::EntityFactory::_clone_name_type arg)
  {
    msg_.clone_name = std::move(arg);
    return Init_EntityFactory_pose(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityFactory msg_;
};

class Init_EntityFactory_sdf_filename
{
public:
  explicit Init_EntityFactory_sdf_filename(::ros_gz_interfaces::msg::EntityFactory & msg)
  : msg_(msg)
  {}
  Init_EntityFactory_clone_name sdf_filename(::ros_gz_interfaces::msg::EntityFactory::_sdf_filename_type arg)
  {
    msg_.sdf_filename = std::move(arg);
    return Init_EntityFactory_clone_name(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityFactory msg_;
};

class Init_EntityFactory_sdf
{
public:
  explicit Init_EntityFactory_sdf(::ros_gz_interfaces::msg::EntityFactory & msg)
  : msg_(msg)
  {}
  Init_EntityFactory_sdf_filename sdf(::ros_gz_interfaces::msg::EntityFactory::_sdf_type arg)
  {
    msg_.sdf = std::move(arg);
    return Init_EntityFactory_sdf_filename(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityFactory msg_;
};

class Init_EntityFactory_allow_renaming
{
public:
  explicit Init_EntityFactory_allow_renaming(::ros_gz_interfaces::msg::EntityFactory & msg)
  : msg_(msg)
  {}
  Init_EntityFactory_sdf allow_renaming(::ros_gz_interfaces::msg::EntityFactory::_allow_renaming_type arg)
  {
    msg_.allow_renaming = std::move(arg);
    return Init_EntityFactory_sdf(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityFactory msg_;
};

class Init_EntityFactory_name
{
public:
  Init_EntityFactory_name()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EntityFactory_allow_renaming name(::ros_gz_interfaces::msg::EntityFactory::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_EntityFactory_allow_renaming(msg_);
  }

private:
  ::ros_gz_interfaces::msg::EntityFactory msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::EntityFactory>()
{
  return ros_gz_interfaces::msg::builder::Init_EntityFactory_name();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__BUILDER_HPP_
