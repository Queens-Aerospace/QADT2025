// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4_msgs:msg/RoverAckermannGuidanceStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__BUILDER_HPP_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4_msgs/msg/detail/rover_ackermann_guidance_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4_msgs
{

namespace msg
{

namespace builder
{

class Init_RoverAckermannGuidanceStatus_heading_error
{
public:
  explicit Init_RoverAckermannGuidanceStatus_heading_error(::px4_msgs::msg::RoverAckermannGuidanceStatus & msg)
  : msg_(msg)
  {}
  ::px4_msgs::msg::RoverAckermannGuidanceStatus heading_error(::px4_msgs::msg::RoverAckermannGuidanceStatus::_heading_error_type arg)
  {
    msg_.heading_error = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannGuidanceStatus msg_;
};

class Init_RoverAckermannGuidanceStatus_lookahead_distance
{
public:
  explicit Init_RoverAckermannGuidanceStatus_lookahead_distance(::px4_msgs::msg::RoverAckermannGuidanceStatus & msg)
  : msg_(msg)
  {}
  Init_RoverAckermannGuidanceStatus_heading_error lookahead_distance(::px4_msgs::msg::RoverAckermannGuidanceStatus::_lookahead_distance_type arg)
  {
    msg_.lookahead_distance = std::move(arg);
    return Init_RoverAckermannGuidanceStatus_heading_error(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannGuidanceStatus msg_;
};

class Init_RoverAckermannGuidanceStatus_timestamp
{
public:
  Init_RoverAckermannGuidanceStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RoverAckermannGuidanceStatus_lookahead_distance timestamp(::px4_msgs::msg::RoverAckermannGuidanceStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_RoverAckermannGuidanceStatus_lookahead_distance(msg_);
  }

private:
  ::px4_msgs::msg::RoverAckermannGuidanceStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4_msgs::msg::RoverAckermannGuidanceStatus>()
{
  return px4_msgs::msg::builder::Init_RoverAckermannGuidanceStatus_timestamp();
}

}  // namespace px4_msgs

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_GUIDANCE_STATUS__BUILDER_HPP_
