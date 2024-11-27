// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/AirspeedValidated.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__AIRSPEED_VALIDATED__BUILDER_HPP_
#define PX4__MSG__DETAIL__AIRSPEED_VALIDATED__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/airspeed_validated__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_AirspeedValidated_pitch_filtered
{
public:
  explicit Init_AirspeedValidated_pitch_filtered(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  ::px4::msg::AirspeedValidated pitch_filtered(::px4::msg::AirspeedValidated::_pitch_filtered_type arg)
  {
    msg_.pitch_filtered = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_throttle_filtered
{
public:
  explicit Init_AirspeedValidated_throttle_filtered(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_pitch_filtered throttle_filtered(::px4::msg::AirspeedValidated::_throttle_filtered_type arg)
  {
    msg_.throttle_filtered = std::move(arg);
    return Init_AirspeedValidated_pitch_filtered(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_airspeed_derivative_filtered
{
public:
  explicit Init_AirspeedValidated_airspeed_derivative_filtered(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_throttle_filtered airspeed_derivative_filtered(::px4::msg::AirspeedValidated::_airspeed_derivative_filtered_type arg)
  {
    msg_.airspeed_derivative_filtered = std::move(arg);
    return Init_AirspeedValidated_throttle_filtered(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_selected_airspeed_index
{
public:
  explicit Init_AirspeedValidated_selected_airspeed_index(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_airspeed_derivative_filtered selected_airspeed_index(::px4::msg::AirspeedValidated::_selected_airspeed_index_type arg)
  {
    msg_.selected_airspeed_index = std::move(arg);
    return Init_AirspeedValidated_airspeed_derivative_filtered(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_airspeed_sensor_measurement_valid
{
public:
  explicit Init_AirspeedValidated_airspeed_sensor_measurement_valid(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_selected_airspeed_index airspeed_sensor_measurement_valid(::px4::msg::AirspeedValidated::_airspeed_sensor_measurement_valid_type arg)
  {
    msg_.airspeed_sensor_measurement_valid = std::move(arg);
    return Init_AirspeedValidated_selected_airspeed_index(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_true_ground_minus_wind_m_s
{
public:
  explicit Init_AirspeedValidated_true_ground_minus_wind_m_s(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_airspeed_sensor_measurement_valid true_ground_minus_wind_m_s(::px4::msg::AirspeedValidated::_true_ground_minus_wind_m_s_type arg)
  {
    msg_.true_ground_minus_wind_m_s = std::move(arg);
    return Init_AirspeedValidated_airspeed_sensor_measurement_valid(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_calibrated_ground_minus_wind_m_s
{
public:
  explicit Init_AirspeedValidated_calibrated_ground_minus_wind_m_s(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_true_ground_minus_wind_m_s calibrated_ground_minus_wind_m_s(::px4::msg::AirspeedValidated::_calibrated_ground_minus_wind_m_s_type arg)
  {
    msg_.calibrated_ground_minus_wind_m_s = std::move(arg);
    return Init_AirspeedValidated_true_ground_minus_wind_m_s(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_true_airspeed_m_s
{
public:
  explicit Init_AirspeedValidated_true_airspeed_m_s(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_calibrated_ground_minus_wind_m_s true_airspeed_m_s(::px4::msg::AirspeedValidated::_true_airspeed_m_s_type arg)
  {
    msg_.true_airspeed_m_s = std::move(arg);
    return Init_AirspeedValidated_calibrated_ground_minus_wind_m_s(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_calibrated_airspeed_m_s
{
public:
  explicit Init_AirspeedValidated_calibrated_airspeed_m_s(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_true_airspeed_m_s calibrated_airspeed_m_s(::px4::msg::AirspeedValidated::_calibrated_airspeed_m_s_type arg)
  {
    msg_.calibrated_airspeed_m_s = std::move(arg);
    return Init_AirspeedValidated_true_airspeed_m_s(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_indicated_airspeed_m_s
{
public:
  explicit Init_AirspeedValidated_indicated_airspeed_m_s(::px4::msg::AirspeedValidated & msg)
  : msg_(msg)
  {}
  Init_AirspeedValidated_calibrated_airspeed_m_s indicated_airspeed_m_s(::px4::msg::AirspeedValidated::_indicated_airspeed_m_s_type arg)
  {
    msg_.indicated_airspeed_m_s = std::move(arg);
    return Init_AirspeedValidated_calibrated_airspeed_m_s(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

class Init_AirspeedValidated_timestamp
{
public:
  Init_AirspeedValidated_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AirspeedValidated_indicated_airspeed_m_s timestamp(::px4::msg::AirspeedValidated::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_AirspeedValidated_indicated_airspeed_m_s(msg_);
  }

private:
  ::px4::msg::AirspeedValidated msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::AirspeedValidated>()
{
  return px4::msg::builder::Init_AirspeedValidated_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__AIRSPEED_VALIDATED__BUILDER_HPP_
