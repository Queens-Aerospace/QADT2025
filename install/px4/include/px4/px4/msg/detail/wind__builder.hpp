// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/Wind.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__WIND__BUILDER_HPP_
#define PX4__MSG__DETAIL__WIND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/wind__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_Wind_beta_innov_var
{
public:
  explicit Init_Wind_beta_innov_var(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  ::px4::msg::Wind beta_innov_var(::px4::msg::Wind::_beta_innov_var_type arg)
  {
    msg_.beta_innov_var = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_beta_innov
{
public:
  explicit Init_Wind_beta_innov(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_beta_innov_var beta_innov(::px4::msg::Wind::_beta_innov_type arg)
  {
    msg_.beta_innov = std::move(arg);
    return Init_Wind_beta_innov_var(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_tas_innov_var
{
public:
  explicit Init_Wind_tas_innov_var(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_beta_innov tas_innov_var(::px4::msg::Wind::_tas_innov_var_type arg)
  {
    msg_.tas_innov_var = std::move(arg);
    return Init_Wind_beta_innov(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_tas_innov
{
public:
  explicit Init_Wind_tas_innov(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_tas_innov_var tas_innov(::px4::msg::Wind::_tas_innov_type arg)
  {
    msg_.tas_innov = std::move(arg);
    return Init_Wind_tas_innov_var(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_variance_east
{
public:
  explicit Init_Wind_variance_east(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_tas_innov variance_east(::px4::msg::Wind::_variance_east_type arg)
  {
    msg_.variance_east = std::move(arg);
    return Init_Wind_tas_innov(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_variance_north
{
public:
  explicit Init_Wind_variance_north(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_variance_east variance_north(::px4::msg::Wind::_variance_north_type arg)
  {
    msg_.variance_north = std::move(arg);
    return Init_Wind_variance_east(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_windspeed_east
{
public:
  explicit Init_Wind_windspeed_east(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_variance_north windspeed_east(::px4::msg::Wind::_windspeed_east_type arg)
  {
    msg_.windspeed_east = std::move(arg);
    return Init_Wind_variance_north(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_windspeed_north
{
public:
  explicit Init_Wind_windspeed_north(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_windspeed_east windspeed_north(::px4::msg::Wind::_windspeed_north_type arg)
  {
    msg_.windspeed_north = std::move(arg);
    return Init_Wind_windspeed_east(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_timestamp_sample
{
public:
  explicit Init_Wind_timestamp_sample(::px4::msg::Wind & msg)
  : msg_(msg)
  {}
  Init_Wind_windspeed_north timestamp_sample(::px4::msg::Wind::_timestamp_sample_type arg)
  {
    msg_.timestamp_sample = std::move(arg);
    return Init_Wind_windspeed_north(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

class Init_Wind_timestamp
{
public:
  Init_Wind_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Wind_timestamp_sample timestamp(::px4::msg::Wind::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_Wind_timestamp_sample(msg_);
  }

private:
  ::px4::msg::Wind msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::Wind>()
{
  return px4::msg::builder::Init_Wind_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__WIND__BUILDER_HPP_
