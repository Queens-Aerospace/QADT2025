// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/VideoRecord.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/video_record__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_VideoRecord_save_filename
{
public:
  explicit Init_VideoRecord_save_filename(::ros_gz_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::VideoRecord save_filename(::ros_gz_interfaces::msg::VideoRecord::_save_filename_type arg)
  {
    msg_.save_filename = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_format
{
public:
  explicit Init_VideoRecord_format(::ros_gz_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  Init_VideoRecord_save_filename format(::ros_gz_interfaces::msg::VideoRecord::_format_type arg)
  {
    msg_.format = std::move(arg);
    return Init_VideoRecord_save_filename(msg_);
  }

private:
  ::ros_gz_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_stop
{
public:
  explicit Init_VideoRecord_stop(::ros_gz_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  Init_VideoRecord_format stop(::ros_gz_interfaces::msg::VideoRecord::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return Init_VideoRecord_format(msg_);
  }

private:
  ::ros_gz_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_start
{
public:
  explicit Init_VideoRecord_start(::ros_gz_interfaces::msg::VideoRecord & msg)
  : msg_(msg)
  {}
  Init_VideoRecord_stop start(::ros_gz_interfaces::msg::VideoRecord::_start_type arg)
  {
    msg_.start = std::move(arg);
    return Init_VideoRecord_stop(msg_);
  }

private:
  ::ros_gz_interfaces::msg::VideoRecord msg_;
};

class Init_VideoRecord_header
{
public:
  Init_VideoRecord_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VideoRecord_start header(::ros_gz_interfaces::msg::VideoRecord::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_VideoRecord_start(msg_);
  }

private:
  ::ros_gz_interfaces::msg::VideoRecord msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::VideoRecord>()
{
  return ros_gz_interfaces::msg::builder::Init_VideoRecord_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__BUILDER_HPP_
