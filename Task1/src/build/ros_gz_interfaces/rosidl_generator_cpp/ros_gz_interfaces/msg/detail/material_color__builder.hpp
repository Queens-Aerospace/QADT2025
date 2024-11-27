// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/MaterialColor.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/material_color__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_MaterialColor_entity_match
{
public:
  explicit Init_MaterialColor_entity_match(::ros_gz_interfaces::msg::MaterialColor & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::MaterialColor entity_match(::ros_gz_interfaces::msg::MaterialColor::_entity_match_type arg)
  {
    msg_.entity_match = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

class Init_MaterialColor_shininess
{
public:
  explicit Init_MaterialColor_shininess(::ros_gz_interfaces::msg::MaterialColor & msg)
  : msg_(msg)
  {}
  Init_MaterialColor_entity_match shininess(::ros_gz_interfaces::msg::MaterialColor::_shininess_type arg)
  {
    msg_.shininess = std::move(arg);
    return Init_MaterialColor_entity_match(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

class Init_MaterialColor_emissive
{
public:
  explicit Init_MaterialColor_emissive(::ros_gz_interfaces::msg::MaterialColor & msg)
  : msg_(msg)
  {}
  Init_MaterialColor_shininess emissive(::ros_gz_interfaces::msg::MaterialColor::_emissive_type arg)
  {
    msg_.emissive = std::move(arg);
    return Init_MaterialColor_shininess(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

class Init_MaterialColor_specular
{
public:
  explicit Init_MaterialColor_specular(::ros_gz_interfaces::msg::MaterialColor & msg)
  : msg_(msg)
  {}
  Init_MaterialColor_emissive specular(::ros_gz_interfaces::msg::MaterialColor::_specular_type arg)
  {
    msg_.specular = std::move(arg);
    return Init_MaterialColor_emissive(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

class Init_MaterialColor_diffuse
{
public:
  explicit Init_MaterialColor_diffuse(::ros_gz_interfaces::msg::MaterialColor & msg)
  : msg_(msg)
  {}
  Init_MaterialColor_specular diffuse(::ros_gz_interfaces::msg::MaterialColor::_diffuse_type arg)
  {
    msg_.diffuse = std::move(arg);
    return Init_MaterialColor_specular(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

class Init_MaterialColor_ambient
{
public:
  explicit Init_MaterialColor_ambient(::ros_gz_interfaces::msg::MaterialColor & msg)
  : msg_(msg)
  {}
  Init_MaterialColor_diffuse ambient(::ros_gz_interfaces::msg::MaterialColor::_ambient_type arg)
  {
    msg_.ambient = std::move(arg);
    return Init_MaterialColor_diffuse(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

class Init_MaterialColor_entity
{
public:
  explicit Init_MaterialColor_entity(::ros_gz_interfaces::msg::MaterialColor & msg)
  : msg_(msg)
  {}
  Init_MaterialColor_ambient entity(::ros_gz_interfaces::msg::MaterialColor::_entity_type arg)
  {
    msg_.entity = std::move(arg);
    return Init_MaterialColor_ambient(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

class Init_MaterialColor_header
{
public:
  Init_MaterialColor_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MaterialColor_entity header(::ros_gz_interfaces::msg::MaterialColor::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_MaterialColor_entity(msg_);
  }

private:
  ::ros_gz_interfaces::msg::MaterialColor msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::MaterialColor>()
{
  return ros_gz_interfaces::msg::builder::Init_MaterialColor_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__BUILDER_HPP_
