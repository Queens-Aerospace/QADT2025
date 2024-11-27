// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ros_gz_interfaces:msg/Contacts.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__CONTACTS__BUILDER_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__CONTACTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ros_gz_interfaces/msg/detail/contacts__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ros_gz_interfaces
{

namespace msg
{

namespace builder
{

class Init_Contacts_contacts
{
public:
  explicit Init_Contacts_contacts(::ros_gz_interfaces::msg::Contacts & msg)
  : msg_(msg)
  {}
  ::ros_gz_interfaces::msg::Contacts contacts(::ros_gz_interfaces::msg::Contacts::_contacts_type arg)
  {
    msg_.contacts = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Contacts msg_;
};

class Init_Contacts_header
{
public:
  Init_Contacts_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Contacts_contacts header(::ros_gz_interfaces::msg::Contacts::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_Contacts_contacts(msg_);
  }

private:
  ::ros_gz_interfaces::msg::Contacts msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ros_gz_interfaces::msg::Contacts>()
{
  return ros_gz_interfaces::msg::builder::Init_Contacts_header();
}

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__CONTACTS__BUILDER_HPP_
