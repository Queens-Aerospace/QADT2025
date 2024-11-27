// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros_gz_interfaces:msg/ParamVec.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_gz_interfaces/msg/detail/param_vec__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace ros_gz_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ParamVec_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_gz_interfaces::msg::ParamVec(_init);
}

void ParamVec_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_gz_interfaces::msg::ParamVec *>(message_memory);
  typed_message->~ParamVec();
}

size_t size_function__ParamVec__params(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<rcl_interfaces::msg::Parameter> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ParamVec__params(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<rcl_interfaces::msg::Parameter> *>(untyped_member);
  return &member[index];
}

void * get_function__ParamVec__params(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<rcl_interfaces::msg::Parameter> *>(untyped_member);
  return &member[index];
}

void fetch_function__ParamVec__params(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const rcl_interfaces::msg::Parameter *>(
    get_const_function__ParamVec__params(untyped_member, index));
  auto & value = *reinterpret_cast<rcl_interfaces::msg::Parameter *>(untyped_value);
  value = item;
}

void assign_function__ParamVec__params(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<rcl_interfaces::msg::Parameter *>(
    get_function__ParamVec__params(untyped_member, index));
  const auto & value = *reinterpret_cast<const rcl_interfaces::msg::Parameter *>(untyped_value);
  item = value;
}

void resize_function__ParamVec__params(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<rcl_interfaces::msg::Parameter> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ParamVec_message_member_array[2] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces::msg::ParamVec, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "params",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<rcl_interfaces::msg::Parameter>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces::msg::ParamVec, params),  // bytes offset in struct
    nullptr,  // default value
    size_function__ParamVec__params,  // size() function pointer
    get_const_function__ParamVec__params,  // get_const(index) function pointer
    get_function__ParamVec__params,  // get(index) function pointer
    fetch_function__ParamVec__params,  // fetch(index, &value) function pointer
    assign_function__ParamVec__params,  // assign(index, value) function pointer
    resize_function__ParamVec__params  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ParamVec_message_members = {
  "ros_gz_interfaces::msg",  // message namespace
  "ParamVec",  // message name
  2,  // number of fields
  sizeof(ros_gz_interfaces::msg::ParamVec),
  ParamVec_message_member_array,  // message members
  ParamVec_init_function,  // function to initialize message memory (memory has to be allocated)
  ParamVec_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ParamVec_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ParamVec_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace ros_gz_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_gz_interfaces::msg::ParamVec>()
{
  return &::ros_gz_interfaces::msg::rosidl_typesupport_introspection_cpp::ParamVec_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_gz_interfaces, msg, ParamVec)() {
  return &::ros_gz_interfaces::msg::rosidl_typesupport_introspection_cpp::ParamVec_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
