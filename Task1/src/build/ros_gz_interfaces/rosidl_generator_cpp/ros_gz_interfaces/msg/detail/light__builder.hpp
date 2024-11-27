// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/light__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Light_intensity
{
public:
  explicit Init_Light_intensity(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::Light intensity(::ros_gz_interfaces::msg::Light::_intensity_type arg)
  {
    msg_.intensity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_parent_id
{
public:
  explicit Init_Light_parent_id(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_intensity parent_id(::ros_gz_interfaces::msg::Light::_parent_id_type arg)
  {
    msg_.parent_id = std::move(arg);
    return Init_Light_intensity(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_id
{
public:
  explicit Init_Light_id(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_parent_id id(::ros_gz_interfaces::msg::Light::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_Light_parent_id(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_spot_falloff
{
public:
  explicit Init_Light_spot_falloff(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_id spot_falloff(::ros_gz_interfaces::msg::Light::_spot_falloff_type arg)
  {
    msg_.spot_falloff = std::move(arg);
    return Init_Light_id(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_spot_outer_angle
{
public:
  explicit Init_Light_spot_outer_angle(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_spot_falloff spot_outer_angle(::ros_gz_interfaces::msg::Light::_spot_outer_angle_type arg)
  {
    msg_.spot_outer_angle = std::move(arg);
    return Init_Light_spot_falloff(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_spot_inner_angle
{
public:
  explicit Init_Light_spot_inner_angle(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_spot_outer_angle spot_inner_angle(::ros_gz_interfaces::msg::Light::_spot_inner_angle_type arg)
  {
    msg_.spot_inner_angle = std::move(arg);
    return Init_Light_spot_outer_angle(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_cast_shadows
{
public:
  explicit Init_Light_cast_shadows(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_spot_inner_angle cast_shadows(::ros_gz_interfaces::msg::Light::_cast_shadows_type arg)
  {
    msg_.cast_shadows = std::move(arg);
    return Init_Light_spot_inner_angle(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_range
{
public:
  explicit Init_Light_range(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_cast_shadows range(::ros_gz_interfaces::msg::Light::_range_type arg)
  {
    msg_.range = std::move(arg);
    return Init_Light_cast_shadows(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_direction
{
public:
  explicit Init_Light_direction(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_range direction(::ros_gz_interfaces::msg::Light::_direction_type arg)
  {
    msg_.direction = std::move(arg);
    return Init_Light_range(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_attenuation_quadratic
{
public:
  explicit Init_Light_attenuation_quadratic(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_direction attenuation_quadratic(::ros_gz_interfaces::msg::Light::_attenuation_quadratic_type arg)
  {
    msg_.attenuation_quadratic = std::move(arg);
    return Init_Light_direction(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_attenuation_linear
{
public:
  explicit Init_Light_attenuation_linear(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_attenuation_quadratic attenuation_linear(::ros_gz_interfaces::msg::Light::_attenuation_linear_type arg)
  {
    msg_.attenuation_linear = std::move(arg);
    return Init_Light_attenuation_quadratic(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_attenuation_constant
{
public:
  explicit Init_Light_attenuation_constant(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_attenuation_linear attenuation_constant(::ros_gz_interfaces::msg::Light::_attenuation_constant_type arg)
  {
    msg_.attenuation_constant = std::move(arg);
    return Init_Light_attenuation_linear(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_specular
{
public:
  explicit Init_Light_specular(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_attenuation_constant specular(::ros_gz_interfaces::msg::Light::_specular_type arg)
  {
    msg_.specular = std::move(arg);
    return Init_Light_attenuation_constant(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_diffuse
{
public:
  explicit Init_Light_diffuse(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_specular diffuse(::ros_gz_interfaces::msg::Light::_diffuse_type arg)
  {
    msg_.diffuse = std::move(arg);
    return Init_Light_specular(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_pose
{
public:
  explicit Init_Light_pose(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_diffuse pose(::ros_gz_interfaces::msg::Light::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return Init_Light_diffuse(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_type
{
public:
  explicit Init_Light_type(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_pose type(::ros_gz_interfaces::msg::Light::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_Light_pose(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_name
{
public:
  explicit Init_Light_name(::ros_gz_interfaces::msg::Light & msg)
  : msg_(msg)
  {}
  Init_Light_type name(::ros_gz_interfaces::msg::Light::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_Light_type(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

class Init_Light_header
{
public:
  Init_Light_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Light_name header(::ros_gz_interfaces::msg::Light::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Light_name(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Light msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::Light>()
{
  return ros_gz_interfaces::msg::builder::Init_Light_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__BUILDER_HPP_
