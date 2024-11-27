// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/MagWorkerData.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MAG_WORKER_DATA__BUILDER_HPP_
#define PX4__MSG__DETAIL__MAG_WORKER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/mag_worker_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_MagWorkerData_z
{
public:
  explicit Init_MagWorkerData_z(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  ::px4::msg::MagWorkerData z(::px4::msg::MagWorkerData::_z_type arg)
  {
    msg_.z = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_y
{
public:
  explicit Init_MagWorkerData_y(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_z y(::px4::msg::MagWorkerData::_y_type arg)
  {
    msg_.y = std::move(arg);
    return Init_MagWorkerData_z(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_x
{
public:
  explicit Init_MagWorkerData_x(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_y x(::px4::msg::MagWorkerData::_x_type arg)
  {
    msg_.x = std::move(arg);
    return Init_MagWorkerData_y(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_side_data_collected
{
public:
  explicit Init_MagWorkerData_side_data_collected(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_x side_data_collected(::px4::msg::MagWorkerData::_side_data_collected_type arg)
  {
    msg_.side_data_collected = std::move(arg);
    return Init_MagWorkerData_x(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_calibration_counter_total
{
public:
  explicit Init_MagWorkerData_calibration_counter_total(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_side_data_collected calibration_counter_total(::px4::msg::MagWorkerData::_calibration_counter_total_type arg)
  {
    msg_.calibration_counter_total = std::move(arg);
    return Init_MagWorkerData_side_data_collected(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_calibration_interval_perside_us
{
public:
  explicit Init_MagWorkerData_calibration_interval_perside_us(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_calibration_counter_total calibration_interval_perside_us(::px4::msg::MagWorkerData::_calibration_interval_perside_us_type arg)
  {
    msg_.calibration_interval_perside_us = std::move(arg);
    return Init_MagWorkerData_calibration_counter_total(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_calibration_points_perside
{
public:
  explicit Init_MagWorkerData_calibration_points_perside(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_calibration_interval_perside_us calibration_points_perside(::px4::msg::MagWorkerData::_calibration_points_perside_type arg)
  {
    msg_.calibration_points_perside = std::move(arg);
    return Init_MagWorkerData_calibration_interval_perside_us(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_done_count
{
public:
  explicit Init_MagWorkerData_done_count(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_calibration_points_perside done_count(::px4::msg::MagWorkerData::_done_count_type arg)
  {
    msg_.done_count = std::move(arg);
    return Init_MagWorkerData_calibration_points_perside(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_timestamp_sample
{
public:
  explicit Init_MagWorkerData_timestamp_sample(::px4::msg::MagWorkerData & msg)
  : msg_(msg)
  {}
  Init_MagWorkerData_done_count timestamp_sample(::px4::msg::MagWorkerData::_timestamp_sample_type arg)
  {
    msg_.timestamp_sample = std::move(arg);
    return Init_MagWorkerData_done_count(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

class Init_MagWorkerData_timestamp
{
public:
  Init_MagWorkerData_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MagWorkerData_timestamp_sample timestamp(::px4::msg::MagWorkerData::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_MagWorkerData_timestamp_sample(msg_);
  }

private:
  ::px4::msg::MagWorkerData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::MagWorkerData>()
{
  return px4::msg::builder::Init_MagWorkerData_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__MAG_WORKER_DATA__BUILDER_HPP_
