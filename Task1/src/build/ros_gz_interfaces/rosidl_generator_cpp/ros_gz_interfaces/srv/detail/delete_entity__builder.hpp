// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:srv/DeleteEntity.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__SRV__DETAIL__DELETE_ENTITY__BUILDER_HPP_
#define ROS_GZ_INTERFACES__SRV__DETAIL__DELETE_ENTITY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/srv/detail/delete_entity__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace srv
{

namespace builder
{

class Init_DeleteEntity_Request_entity
{
public:
  Init_DeleteEntity_Request_entity()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros_gz_interfaces::srv::DeleteEntity_Request entity(::ros_gz_interfaces::srv::DeleteEntity_Request::_entity_type arg)
  {
    msg_.entity = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::srv::DeleteEntity_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::srv::DeleteEntity_Request>()
{
  return ros_gz_interfaces::srv::builder::Init_DeleteEntity_Request_entity();
}

}  // namespace ros_gz_interfaces


namespace ros_gz_interfaces
{

namespace srv
{

namespace builder
{

class Init_DeleteEntity_Response_success
{
public:
  Init_DeleteEntity_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ros_gz_interfaces::srv::DeleteEntity_Response success(::ros_gz_interfaces::srv::DeleteEntity_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::srv::DeleteEntity_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::srv::DeleteEntity_Response>()
{
  return ros_gz_interfaces::srv::builder::Init_DeleteEntity_Response_success();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__SRV__DETAIL__DELETE_ENTITY__BUILDER_HPP_
