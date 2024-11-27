// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/Float32Array.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/float32_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Float32Array_data
{
public:
  Init_Float32Array_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros_gz_interfaces::msg::Float32Array data(::ros_gz_interfaces::msg::Float32Array::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Float32Array msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::Float32Array>()
{
  return ros_gz_interfaces::msg::builder::Init_Float32Array_data();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__BUILDER_HPP_
