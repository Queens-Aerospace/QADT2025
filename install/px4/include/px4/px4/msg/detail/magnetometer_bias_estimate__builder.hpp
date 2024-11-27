// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/MagnetometerBiasEstimate.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MAGNETOMETER_BIAS_ESTIMATE__BUILDER_HPP_
#define PX4__MSG__DETAIL__MAGNETOMETER_BIAS_ESTIMATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/magnetometer_bias_estimate__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_MagnetometerBiasEstimate_stable
{
public:
  explicit Init_MagnetometerBiasEstimate_stable(::px4::msg::MagnetometerBiasEstimate & msg)
  : msg_(msg)
  {}
  ::px4::msg::MagnetometerBiasEstimate stable(::px4::msg::MagnetometerBiasEstimate::_stable_type arg)
  {
    msg_.stable = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::MagnetometerBiasEstimate msg_;
};

class Init_MagnetometerBiasEstimate_valid
{
public:
  explicit Init_MagnetometerBiasEstimate_valid(::px4::msg::MagnetometerBiasEstimate & msg)
  : msg_(msg)
  {}
  Init_MagnetometerBiasEstimate_stable valid(::px4::msg::MagnetometerBiasEstimate::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_MagnetometerBiasEstimate_stable(msg_);
  }

private:
  ::px4::msg::MagnetometerBiasEstimate msg_;
};

class Init_MagnetometerBiasEstimate_bias_z
{
public:
  explicit Init_MagnetometerBiasEstimate_bias_z(::px4::msg::MagnetometerBiasEstimate & msg)
  : msg_(msg)
  {}
  Init_MagnetometerBiasEstimate_valid bias_z(::px4::msg::MagnetometerBiasEstimate::_bias_z_type arg)
  {
    msg_.bias_z = std::move(arg);
    return Init_MagnetometerBiasEstimate_valid(msg_);
  }

private:
  ::px4::msg::MagnetometerBiasEstimate msg_;
};

class Init_MagnetometerBiasEstimate_bias_y
{
public:
  explicit Init_MagnetometerBiasEstimate_bias_y(::px4::msg::MagnetometerBiasEstimate & msg)
  : msg_(msg)
  {}
  Init_MagnetometerBiasEstimate_bias_z bias_y(::px4::msg::MagnetometerBiasEstimate::_bias_y_type arg)
  {
    msg_.bias_y = std::move(arg);
    return Init_MagnetometerBiasEstimate_bias_z(msg_);
  }

private:
  ::px4::msg::MagnetometerBiasEstimate msg_;
};

class Init_MagnetometerBiasEstimate_bias_x
{
public:
  explicit Init_MagnetometerBiasEstimate_bias_x(::px4::msg::MagnetometerBiasEstimate & msg)
  : msg_(msg)
  {}
  Init_MagnetometerBiasEstimate_bias_y bias_x(::px4::msg::MagnetometerBiasEstimate::_bias_x_type arg)
  {
    msg_.bias_x = std::move(arg);
    return Init_MagnetometerBiasEstimate_bias_y(msg_);
  }

private:
  ::px4::msg::MagnetometerBiasEstimate msg_;
};

class Init_MagnetometerBiasEstimate_timestamp
{
public:
  Init_MagnetometerBiasEstimate_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MagnetometerBiasEstimate_bias_x timestamp(::px4::msg::MagnetometerBiasEstimate::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_MagnetometerBiasEstimate_bias_x(msg_);
  }

private:
  ::px4::msg::MagnetometerBiasEstimate msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::MagnetometerBiasEstimate>()
{
  return px4::msg::builder::Init_MagnetometerBiasEstimate_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__MAGNETOMETER_BIAS_ESTIMATE__BUILDER_HPP_
