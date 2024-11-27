// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/altimeter__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Altimeter_vertical_reference
{
public:
  explicit Init_Altimeter_vertical_reference(::ros_gz_interfaces::msg::Altimeter & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::Altimeter vertical_reference(::ros_gz_interfaces::msg::Altimeter::_vertical_reference_type arg)
  {
    msg_.vertical_reference = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Altimeter msg_;
};

class Init_Altimeter_vertical_velocity
{
public:
  explicit Init_Altimeter_vertical_velocity(::ros_gz_interfaces::msg::Altimeter & msg)
  : msg_(msg)
  {}
  Init_Altimeter_vertical_reference vertical_velocity(::ros_gz_interfaces::msg::Altimeter::_vertical_velocity_type arg)
  {
    msg_.vertical_velocity = std::move(arg);
    return Init_Altimeter_vertical_reference(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Altimeter msg_;
};

class Init_Altimeter_vertical_position
{
public:
  explicit Init_Altimeter_vertical_position(::ros_gz_interfaces::msg::Altimeter & msg)
  : msg_(msg)
  {}
  Init_Altimeter_vertical_velocity vertical_position(::ros_gz_interfaces::msg::Altimeter::_vertical_position_type arg)
  {
    msg_.vertical_position = std::move(arg);
    return Init_Altimeter_vertical_velocity(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Altimeter msg_;
};

class Init_Altimeter_header
{
public:
  Init_Altimeter_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Altimeter_vertical_position header(::ros_gz_interfaces::msg::Altimeter::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Altimeter_vertical_position(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Altimeter msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::Altimeter>()
{
  return ros_gz_interfaces::msg::builder::Init_Altimeter_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__BUILDER_HPP_
