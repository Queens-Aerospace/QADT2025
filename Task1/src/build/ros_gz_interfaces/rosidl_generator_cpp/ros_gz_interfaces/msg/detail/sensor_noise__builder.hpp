// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/SensorNoise.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/sensor_noise__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_SensorNoise_dynamic_bias_correlation_time
{
public:
  explicit Init_SensorNoise_dynamic_bias_correlation_time(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::SensorNoise dynamic_bias_correlation_time(::ros_gz_interfaces::msg::SensorNoise::_dynamic_bias_correlation_time_type arg)
  {
    msg_.dynamic_bias_correlation_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_dynamic_bias_stddev
{
public:
  explicit Init_SensorNoise_dynamic_bias_stddev(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  Init_SensorNoise_dynamic_bias_correlation_time dynamic_bias_stddev(::ros_gz_interfaces::msg::SensorNoise::_dynamic_bias_stddev_type arg)
  {
    msg_.dynamic_bias_stddev = std::move(arg);
    return Init_SensorNoise_dynamic_bias_correlation_time(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_precision
{
public:
  explicit Init_SensorNoise_precision(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  Init_SensorNoise_dynamic_bias_stddev precision(::ros_gz_interfaces::msg::SensorNoise::_precision_type arg)
  {
    msg_.precision = std::move(arg);
    return Init_SensorNoise_dynamic_bias_stddev(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_bias_stddev
{
public:
  explicit Init_SensorNoise_bias_stddev(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  Init_SensorNoise_precision bias_stddev(::ros_gz_interfaces::msg::SensorNoise::_bias_stddev_type arg)
  {
    msg_.bias_stddev = std::move(arg);
    return Init_SensorNoise_precision(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_bias_mean
{
public:
  explicit Init_SensorNoise_bias_mean(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  Init_SensorNoise_bias_stddev bias_mean(::ros_gz_interfaces::msg::SensorNoise::_bias_mean_type arg)
  {
    msg_.bias_mean = std::move(arg);
    return Init_SensorNoise_bias_stddev(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_stddev
{
public:
  explicit Init_SensorNoise_stddev(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  Init_SensorNoise_bias_mean stddev(::ros_gz_interfaces::msg::SensorNoise::_stddev_type arg)
  {
    msg_.stddev = std::move(arg);
    return Init_SensorNoise_bias_mean(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_mean
{
public:
  explicit Init_SensorNoise_mean(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  Init_SensorNoise_stddev mean(::ros_gz_interfaces::msg::SensorNoise::_mean_type arg)
  {
    msg_.mean = std::move(arg);
    return Init_SensorNoise_stddev(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_type
{
public:
  explicit Init_SensorNoise_type(::ros_gz_interfaces::msg::SensorNoise & msg)
  : msg_(msg)
  {}
  Init_SensorNoise_mean type(::ros_gz_interfaces::msg::SensorNoise::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_SensorNoise_mean(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

class Init_SensorNoise_header
{
public:
  Init_SensorNoise_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_SensorNoise_type header(::ros_gz_interfaces::msg::SensorNoise::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_SensorNoise_type(msg_);
  }

private:
  ::ros_gz_interfaces::msg::SensorNoise msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::SensorNoise>()
{
  return ros_gz_interfaces::msg::builder::Init_SensorNoise_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__BUILDER_HPP_
