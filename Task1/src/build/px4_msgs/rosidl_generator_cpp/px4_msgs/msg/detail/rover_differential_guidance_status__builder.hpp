// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverDifferentialGuidanceStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_GUIDANCE_STATUS__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_GUIDANCE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_differential_guidance_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverDifferentialGuidanceStatus_state_machine
{
public:
  explicit Init_RoverDifferentialGuidanceStatus_state_machine(::px4_msgs::msg::RoverDifferentialGuidanceStatus & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverDifferentialGuidanceStatus state_machine(::px4_msgs::msg::RoverDifferentialGuidanceStatus::_state_machine_type arg)
  {
    msg_.state_machine = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialGuidanceStatus msg_;
};

class Init_RoverDifferentialGuidanceStatus_heading_error_deg
{
public:
  explicit Init_RoverDifferentialGuidanceStatus_heading_error_deg(::px4_msgs::msg::RoverDifferentialGuidanceStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialGuidanceStatus_state_machine heading_error_deg(::px4_msgs::msg::RoverDifferentialGuidanceStatus::_heading_error_deg_type arg)
  {
    msg_.heading_error_deg = std::move(arg);
    return Init_RoverDifferentialGuidanceStatus_state_machine(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialGuidanceStatus msg_;
};

class Init_RoverDifferentialGuidanceStatus_lookahead_distance
{
public:
  explicit Init_RoverDifferentialGuidanceStatus_lookahead_distance(::px4_msgs::msg::RoverDifferentialGuidanceStatus & msg)
  : msg_(msg)
  {}
  Init_RoverDifferentialGuidanceStatus_heading_error_deg lookahead_distance(::px4_msgs::msg::RoverDifferentialGuidanceStatus::_lookahead_distance_type arg)
  {
    msg_.lookahead_distance = std::move(arg);
    return Init_RoverDifferentialGuidanceStatus_heading_error_deg(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialGuidanceStatus msg_;
};

class Init_RoverDifferentialGuidanceStatus_timestamp
{
public:
  Init_RoverDifferentialGuidanceStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverDifferentialGuidanceStatus_lookahead_distance timestamp(::px4_msgs::msg::RoverDifferentialGuidanceStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverDifferentialGuidanceStatus_lookahead_distance(msg_);
  }

private:
  ::px4_msgs::msg::RoverDifferentialGuidanceStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverDifferentialGuidanceStatus>()
{
  return px4_msgs::msg::builder::Init_RoverDifferentialGuidanceStatus_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_DIFFERENTIAL_GUIDANCE_STATUS__BUILDER_HPP_
