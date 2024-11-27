// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros_gz_interfaces:msg/WorldControl.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/world_control__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros_gz_interfaces/msg/detail/world_control__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace ros_gz_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const ros_gz_interfaces::msg::WorldReset &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  ros_gz_interfaces::msg::WorldReset &);
size_t get_serialized_size(
  const ros_gz_interfaces::msg::WorldReset &,
  size_t current_alignment);
size_t
max_serialized_size_WorldReset(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace ros_gz_interfaces

namespace builtin_interfaces
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const builtin_interfaces::msg::Time &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  builtin_interfaces::msg::Time &);
size_t get_serialized_size(
  const builtin_interfaces::msg::Time &,
  size_t current_alignment);
size_t
max_serialized_size_Time(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace builtin_interfaces


namespace ros_gz_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
cdr_serialize(
  const ros_gz_interfaces::msg::WorldControl & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: pause
  cdr << (ros_message.pause ? true : false);
  // Member: step
  cdr << (ros_message.step ? true : false);
  // Member: multi_step
  cdr << ros_message.multi_step;
  // Member: reset
  ros_gz_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.reset,
    cdr);
  // Member: seed
  cdr << ros_message.seed;
  // Member: run_to_sim_time
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.run_to_sim_time,
    cdr);
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros_gz_interfaces::msg::WorldControl & ros_message)
{
  // Member: pause
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.pause = tmp ? true : false;
  }

  // Member: step
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.step = tmp ? true : false;
  }

  // Member: multi_step
  cdr >> ros_message.multi_step;

  // Member: reset
  ros_gz_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.reset);

  // Member: seed
  cdr >> ros_message.seed;

  // Member: run_to_sim_time
  builtin_interfaces::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.run_to_sim_time);

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
get_serialized_size(
  const ros_gz_interfaces::msg::WorldControl & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: pause
  {
    size_t item_size = sizeof(ros_message.pause);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: step
  {
    size_t item_size = sizeof(ros_message.step);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: multi_step
  {
    size_t item_size = sizeof(ros_message.multi_step);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: reset

  current_alignment +=
    ros_gz_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.reset, current_alignment);
  // Member: seed
  {
    size_t item_size = sizeof(ros_message.seed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: run_to_sim_time

  current_alignment +=
    builtin_interfaces::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.run_to_sim_time, current_alignment);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
max_serialized_size_WorldControl(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: pause
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: step
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: multi_step
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: reset
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        ros_gz_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_WorldReset(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: seed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: run_to_sim_time
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        builtin_interfaces::msg::typesupport_fastrtps_cpp::max_serialized_size_Time(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros_gz_interfaces::msg::WorldControl;
    is_plain =
      (
      offsetof(DataType, run_to_sim_time) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _WorldControl__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros_gz_interfaces::msg::WorldControl *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _WorldControl__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros_gz_interfaces::msg::WorldControl *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _WorldControl__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros_gz_interfaces::msg::WorldControl *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _WorldControl__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_WorldControl(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _WorldControl__callbacks = {
  "ros_gz_interfaces::msg",
  "WorldControl",
  _WorldControl__cdr_serialize,
  _WorldControl__cdr_deserialize,
  _WorldControl__get_serialized_size,
  _WorldControl__max_serialized_size
};

static rosidl_message_type_support_t _WorldControl__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_WorldControl__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros_gz_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros_gz_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_gz_interfaces::msg::WorldControl>()
{
  return &ros_gz_interfaces::msg::typesupport_fastrtps_cpp::_WorldControl__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_gz_interfaces, msg, WorldControl)() {
  return &ros_gz_interfaces::msg::typesupport_fastrtps_cpp::_WorldControl__handle;
}

#ifdef __cplusplus
}
#endif
