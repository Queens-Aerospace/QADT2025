// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from px4_msgs:msg/RoverAckermannSetpoint.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rover_ackermann_setpoint__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "px4_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "px4_msgs/msg/detail/rover_ackermann_setpoint__struct.h"
#include "px4_msgs/msg/detail/rover_ackermann_setpoint__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _RoverAckermannSetpoint__ros_msg_type = px4_msgs__msg__RoverAckermannSetpoint;

static bool _RoverAckermannSetpoint__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RoverAckermannSetpoint__ros_msg_type * ros_message = static_cast<const _RoverAckermannSetpoint__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr << ros_message->timestamp;
  }

  // Field name: forward_speed_setpoint
  {
    cdr << ros_message->forward_speed_setpoint;
  }

  // Field name: forward_speed_setpoint_normalized
  {
    cdr << ros_message->forward_speed_setpoint_normalized;
  }

  // Field name: steering_setpoint
  {
    cdr << ros_message->steering_setpoint;
  }

  // Field name: steering_setpoint_normalized
  {
    cdr << ros_message->steering_setpoint_normalized;
  }

  // Field name: lateral_acceleration_setpoint
  {
    cdr << ros_message->lateral_acceleration_setpoint;
  }

  return true;
}

static bool _RoverAckermannSetpoint__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RoverAckermannSetpoint__ros_msg_type * ros_message = static_cast<_RoverAckermannSetpoint__ros_msg_type *>(untyped_ros_message);
  // Field name: timestamp
  {
    cdr >> ros_message->timestamp;
  }

  // Field name: forward_speed_setpoint
  {
    cdr >> ros_message->forward_speed_setpoint;
  }

  // Field name: forward_speed_setpoint_normalized
  {
    cdr >> ros_message->forward_speed_setpoint_normalized;
  }

  // Field name: steering_setpoint
  {
    cdr >> ros_message->steering_setpoint;
  }

  // Field name: steering_setpoint_normalized
  {
    cdr >> ros_message->steering_setpoint_normalized;
  }

  // Field name: lateral_acceleration_setpoint
  {
    cdr >> ros_message->lateral_acceleration_setpoint;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t get_serialized_size_px4_msgs__msg__RoverAckermannSetpoint(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RoverAckermannSetpoint__ros_msg_type * ros_message = static_cast<const _RoverAckermannSetpoint__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name timestamp
  {
    size_t item_size = sizeof(ros_message->timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name forward_speed_setpoint
  {
    size_t item_size = sizeof(ros_message->forward_speed_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name forward_speed_setpoint_normalized
  {
    size_t item_size = sizeof(ros_message->forward_speed_setpoint_normalized);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name steering_setpoint
  {
    size_t item_size = sizeof(ros_message->steering_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name steering_setpoint_normalized
  {
    size_t item_size = sizeof(ros_message->steering_setpoint_normalized);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name lateral_acceleration_setpoint
  {
    size_t item_size = sizeof(ros_message->lateral_acceleration_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _RoverAckermannSetpoint__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_px4_msgs__msg__RoverAckermannSetpoint(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_px4_msgs
size_t max_serialized_size_px4_msgs__msg__RoverAckermannSetpoint(
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

  // member: timestamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: forward_speed_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: forward_speed_setpoint_normalized
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: steering_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: steering_setpoint_normalized
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: lateral_acceleration_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = px4_msgs__msg__RoverAckermannSetpoint;
    is_plain =
      (
      offsetof(DataType, lateral_acceleration_setpoint) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RoverAckermannSetpoint__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_px4_msgs__msg__RoverAckermannSetpoint(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RoverAckermannSetpoint = {
  "px4_msgs::msg",
  "RoverAckermannSetpoint",
  _RoverAckermannSetpoint__cdr_serialize,
  _RoverAckermannSetpoint__cdr_deserialize,
  _RoverAckermannSetpoint__get_serialized_size,
  _RoverAckermannSetpoint__max_serialized_size
};

static rosidl_message_type_support_t _RoverAckermannSetpoint__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RoverAckermannSetpoint,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, px4_msgs, msg, RoverAckermannSetpoint)() {
  return &_RoverAckermannSetpoint__type_support;
}

#if defined(__cplusplus)
}
#endif
