// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/TrackVisual.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/track_visual__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrackVisual_inherit_yaw
{
public:
  explicit Init_TrackVisual_inherit_yaw(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::TrackVisual inherit_yaw(::ros_gz_interfaces::msg::TrackVisual::_inherit_yaw_type arg)
  {
    msg_.inherit_yaw = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_xyz
{
public:
  explicit Init_TrackVisual_xyz(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_inherit_yaw xyz(::ros_gz_interfaces::msg::TrackVisual::_xyz_type arg)
  {
    msg_.xyz = std::move(arg);
    return Init_TrackVisual_inherit_yaw(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_use_model_frame
{
public:
  explicit Init_TrackVisual_use_model_frame(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_xyz use_model_frame(::ros_gz_interfaces::msg::TrackVisual::_use_model_frame_type arg)
  {
    msg_.use_model_frame = std::move(arg);
    return Init_TrackVisual_xyz(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_is_static
{
public:
  explicit Init_TrackVisual_is_static(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_use_model_frame is_static(::ros_gz_interfaces::msg::TrackVisual::_is_static_type arg)
  {
    msg_.is_static = std::move(arg);
    return Init_TrackVisual_use_model_frame(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_max_dist
{
public:
  explicit Init_TrackVisual_max_dist(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_is_static max_dist(::ros_gz_interfaces::msg::TrackVisual::_max_dist_type arg)
  {
    msg_.max_dist = std::move(arg);
    return Init_TrackVisual_is_static(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_min_dist
{
public:
  explicit Init_TrackVisual_min_dist(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_max_dist min_dist(::ros_gz_interfaces::msg::TrackVisual::_min_dist_type arg)
  {
    msg_.min_dist = std::move(arg);
    return Init_TrackVisual_max_dist(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_inherit_orientation
{
public:
  explicit Init_TrackVisual_inherit_orientation(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_min_dist inherit_orientation(::ros_gz_interfaces::msg::TrackVisual::_inherit_orientation_type arg)
  {
    msg_.inherit_orientation = std::move(arg);
    return Init_TrackVisual_min_dist(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_id
{
public:
  explicit Init_TrackVisual_id(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_inherit_orientation id(::ros_gz_interfaces::msg::TrackVisual::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_TrackVisual_inherit_orientation(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_name
{
public:
  explicit Init_TrackVisual_name(::ros_gz_interfaces::msg::TrackVisual & msg)
  : msg_(msg)
  {}
  Init_TrackVisual_id name(::ros_gz_interfaces::msg::TrackVisual::_name_type arg)
  {
    msg_.name = std::move(arg);
    return Init_TrackVisual_id(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

class Init_TrackVisual_header
{
public:
  Init_TrackVisual_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrackVisual_name header(::ros_gz_interfaces::msg::TrackVisual::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_TrackVisual_name(msg_);
  }

private:
  ::ros_gz_interfaces::msg::TrackVisual msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::TrackVisual>()
{
  return ros_gz_interfaces::msg::builder::Init_TrackVisual_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__BUILDER_HPP_
