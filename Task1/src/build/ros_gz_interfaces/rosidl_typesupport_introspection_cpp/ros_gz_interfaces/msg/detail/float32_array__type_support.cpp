// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from ros_gz_interfaces:msg/Float32Array.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "ros_gz_interfaces/msg/detail/float32_array__struct.hpp"
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

void Float32Array_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) ros_gz_interfaces::msg::Float32Array(_init);
}

void Float32Array_fini_function(void * message_memory)
{
  auto typed_message = static_cast<ros_gz_interfaces::msg::Float32Array *>(message_memory);
  typed_message->~Float32Array();
}

size_t size_function__Float32Array__data(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<float> *>(untyped_member);
  return member->size();
}

const void * get_const_function__Float32Array__data(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<float> *>(untyped_member);
  return &member[index];
}

void * get_function__Float32Array__data(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<float> *>(untyped_member);
  return &member[index];
}

void fetch_function__Float32Array__data(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const float *>(
    get_const_function__Float32Array__data(untyped_member, index));
  auto & value = *reinterpret_cast<float *>(untyped_value);
  value = item;
}

void assign_function__Float32Array__data(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<float *>(
    get_function__Float32Array__data(untyped_member, index));
  const auto & value = *reinterpret_cast<const float *>(untyped_value);
  item = value;
}

void resize_function__Float32Array__data(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<float> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Float32Array_message_member_array[1] = {
  {
    "data",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces::msg::Float32Array, data),  // bytes offset in struct
    nullptr,  // default value
    size_function__Float32Array__data,  // size() function pointer
    get_const_function__Float32Array__data,  // get_const(index) function pointer
    get_function__Float32Array__data,  // get(index) function pointer
    fetch_function__Float32Array__data,  // fetch(index, &value) function pointer
    assign_function__Float32Array__data,  // assign(index, value) function pointer
    resize_function__Float32Array__data  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Float32Array_message_members = {
  "ros_gz_interfaces::msg",  // message namespace
  "Float32Array",  // message name
  1,  // number of fields
  sizeof(ros_gz_interfaces::msg::Float32Array),
  Float32Array_message_member_array,  // message members
  Float32Array_init_function,  // function to initialize message memory (memory has to be allocated)
  Float32Array_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Float32Array_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Float32Array_message_members,
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
get_message_type_support_handle<ros_gz_interfaces::msg::Float32Array>()
{
  return &::ros_gz_interfaces::msg::rosidl_typesupport_introspection_cpp::Float32Array_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, ros_gz_interfaces, msg, Float32Array)() {
  return &::ros_gz_interfaces::msg::rosidl_typesupport_introspection_cpp::Float32Array_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
