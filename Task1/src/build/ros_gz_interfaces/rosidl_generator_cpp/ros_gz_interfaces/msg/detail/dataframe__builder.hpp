// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/Dataframe.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/dataframe__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Dataframe_rssi
{
public:
  explicit Init_Dataframe_rssi(::ros_gz_interfaces::msg::Dataframe & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::Dataframe rssi(::ros_gz_interfaces::msg::Dataframe::_rssi_type arg)
  {
    msg_.rssi = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Dataframe msg_;
};

class Init_Dataframe_data
{
public:
  explicit Init_Dataframe_data(::ros_gz_interfaces::msg::Dataframe & msg)
  : msg_(msg)
  {}
  Init_Dataframe_rssi data(::ros_gz_interfaces::msg::Dataframe::_data_type arg)
  {
    msg_.data = std::move(arg);
    return Init_Dataframe_rssi(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Dataframe msg_;
};

class Init_Dataframe_dst_address
{
public:
  explicit Init_Dataframe_dst_address(::ros_gz_interfaces::msg::Dataframe & msg)
  : msg_(msg)
  {}
  Init_Dataframe_data dst_address(::ros_gz_interfaces::msg::Dataframe::_dst_address_type arg)
  {
    msg_.dst_address = std::move(arg);
    return Init_Dataframe_data(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Dataframe msg_;
};

class Init_Dataframe_src_address
{
public:
  explicit Init_Dataframe_src_address(::ros_gz_interfaces::msg::Dataframe & msg)
  : msg_(msg)
  {}
  Init_Dataframe_dst_address src_address(::ros_gz_interfaces::msg::Dataframe::_src_address_type arg)
  {
    msg_.src_address = std::move(arg);
    return Init_Dataframe_dst_address(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Dataframe msg_;
};

class Init_Dataframe_header
{
public:
  Init_Dataframe_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Dataframe_src_address header(::ros_gz_interfaces::msg::Dataframe::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Dataframe_src_address(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Dataframe msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::Dataframe>()
{
  return ros_gz_interfaces::msg::builder::Init_Dataframe_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__BUILDER_HPP_
