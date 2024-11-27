// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/LaunchDetectionStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__LAUNCH_DETECTION_STATUS__BUILDER_HPP_
#define PX4__MSG__DETAIL__LAUNCH_DETECTION_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/launch_detection_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_LaunchDetectionStatus_launch_detection_state
{
public:
  explicit Init_LaunchDetectionStatus_launch_detection_state(::px4::msg::LaunchDetectionStatus & msg)
  : msg_(msg)
  {}
  ::px4::msg::LaunchDetectionStatus launch_detection_state(::px4::msg::LaunchDetectionStatus::_launch_detection_state_type arg)
  {
    msg_.launch_detection_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::LaunchDetectionStatus msg_;
};

class Init_LaunchDetectionStatus_timestamp
{
public:
  Init_LaunchDetectionStatus_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_LaunchDetectionStatus_launch_detection_state timestamp(::px4::msg::LaunchDetectionStatus::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_LaunchDetectionStatus_launch_detection_state(msg_);
  }

private:
  ::px4::msg::LaunchDetectionStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::LaunchDetectionStatus>()
{
  return px4::msg::builder::Init_LaunchDetectionStatus_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__LAUNCH_DETECTION_STATUS__BUILDER_HPP_
