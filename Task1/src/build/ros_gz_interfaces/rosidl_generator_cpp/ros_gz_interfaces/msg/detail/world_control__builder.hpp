// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/WorldControl.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/world_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_WorldControl_run_to_sim_time
{
public:
  explicit Init_WorldControl_run_to_sim_time(::ros_gz_interfaces::msg::WorldControl & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::WorldControl run_to_sim_time(::ros_gz_interfaces::msg::WorldControl::_run_to_sim_time_type arg)
  {
    msg_.run_to_sim_time = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldControl msg_;
};

class Init_WorldControl_seed
{
public:
  explicit Init_WorldControl_seed(::ros_gz_interfaces::msg::WorldControl & msg)
  : msg_(msg)
  {}
  Init_WorldControl_run_to_sim_time seed(::ros_gz_interfaces::msg::WorldControl::_seed_type arg)
  {
    msg_.seed = std::move(arg);
    return Init_WorldControl_run_to_sim_time(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldControl msg_;
};

class Init_WorldControl_reset
{
public:
  explicit Init_WorldControl_reset(::ros_gz_interfaces::msg::WorldControl & msg)
  : msg_(msg)
  {}
  Init_WorldControl_seed reset(::ros_gz_interfaces::msg::WorldControl::_reset_type arg)
  {
    msg_.reset = std::move(arg);
    return Init_WorldControl_seed(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldControl msg_;
};

class Init_WorldControl_multi_step
{
public:
  explicit Init_WorldControl_multi_step(::ros_gz_interfaces::msg::WorldControl & msg)
  : msg_(msg)
  {}
  Init_WorldControl_reset multi_step(::ros_gz_interfaces::msg::WorldControl::_multi_step_type arg)
  {
    msg_.multi_step = std::move(arg);
    return Init_WorldControl_reset(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldControl msg_;
};

class Init_WorldControl_step
{
public:
  explicit Init_WorldControl_step(::ros_gz_interfaces::msg::WorldControl & msg)
  : msg_(msg)
  {}
  Init_WorldControl_multi_step step(::ros_gz_interfaces::msg::WorldControl::_step_type arg)
  {
    msg_.step = std::move(arg);
    return Init_WorldControl_multi_step(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldControl msg_;
};

class Init_WorldControl_pause
{
public:
  Init_WorldControl_pause()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_WorldControl_step pause(::ros_gz_interfaces::msg::WorldControl::_pause_type arg)
  {
    msg_.pause = std::move(arg);
    return Init_WorldControl_step(msg_);
  }

private:
  ::ros_gz_interfaces::msg::WorldControl msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::WorldControl>()
{
  return ros_gz_interfaces::msg::builder::Init_WorldControl_pause();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__BUILDER_HPP_
