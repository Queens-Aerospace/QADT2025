// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from px4_msgs:msg/RoverDifferentialStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rover_differential_status__rosidl_typesupport_fastrtps_cpp.hpp"
#include "px4_msgs/msg/detail/rover_differential_status__struct.hpp"

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

namespace px4_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_serialize(
  const px4_msgs::msg::RoverDifferentialStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: timestamp
  cdr << ros_message.timestamp;
  // Member: measured_forward_speed
  cdr << ros_message.measured_forward_speed;
  // Member: adjusted_forward_speed_setpoint
  cdr << ros_message.adjusted_forward_speed_setpoint;
  // Member: measured_yaw
  cdr << ros_message.measured_yaw;
  // Member: adjusted_yaw_setpoint
  cdr << ros_message.adjusted_yaw_setpoint;
  // Member: clyaw_yaw_rate_setpoint
  cdr << ros_message.clyaw_yaw_rate_setpoint;
  // Member: measured_yaw_rate
  cdr << ros_message.measured_yaw_rate;
  // Member: adjusted_yaw_rate_setpoint
  cdr << ros_message.adjusted_yaw_rate_setpoint;
  // Member: pid_yaw_integral
  cdr << ros_message.pid_yaw_integral;
  // Member: pid_yaw_rate_integral
  cdr << ros_message.pid_yaw_rate_integral;
  // Member: pid_throttle_integral
  cdr << ros_message.pid_throttle_integral;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  px4_msgs::msg::RoverDifferentialStatus & ros_message)
{
  // Member: timestamp
  cdr >> ros_message.timestamp;

  // Member: measured_forward_speed
  cdr >> ros_message.measured_forward_speed;

  // Member: adjusted_forward_speed_setpoint
  cdr >> ros_message.adjusted_forward_speed_setpoint;

  // Member: measured_yaw
  cdr >> ros_message.measured_yaw;

  // Member: adjusted_yaw_setpoint
  cdr >> ros_message.adjusted_yaw_setpoint;

  // Member: clyaw_yaw_rate_setpoint
  cdr >> ros_message.clyaw_yaw_rate_setpoint;

  // Member: measured_yaw_rate
  cdr >> ros_message.measured_yaw_rate;

  // Member: adjusted_yaw_rate_setpoint
  cdr >> ros_message.adjusted_yaw_rate_setpoint;

  // Member: pid_yaw_integral
  cdr >> ros_message.pid_yaw_integral;

  // Member: pid_yaw_rate_integral
  cdr >> ros_message.pid_yaw_rate_integral;

  // Member: pid_throttle_integral
  cdr >> ros_message.pid_throttle_integral;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
get_serialized_size(
  const px4_msgs::msg::RoverDifferentialStatus & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: timestamp
  {
    size_t item_size = sizeof(ros_message.timestamp);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: measured_forward_speed
  {
    size_t item_size = sizeof(ros_message.measured_forward_speed);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: adjusted_forward_speed_setpoint
  {
    size_t item_size = sizeof(ros_message.adjusted_forward_speed_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: measured_yaw
  {
    size_t item_size = sizeof(ros_message.measured_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: adjusted_yaw_setpoint
  {
    size_t item_size = sizeof(ros_message.adjusted_yaw_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: clyaw_yaw_rate_setpoint
  {
    size_t item_size = sizeof(ros_message.clyaw_yaw_rate_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: measured_yaw_rate
  {
    size_t item_size = sizeof(ros_message.measured_yaw_rate);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: adjusted_yaw_rate_setpoint
  {
    size_t item_size = sizeof(ros_message.adjusted_yaw_rate_setpoint);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pid_yaw_integral
  {
    size_t item_size = sizeof(ros_message.pid_yaw_integral);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pid_yaw_rate_integral
  {
    size_t item_size = sizeof(ros_message.pid_yaw_rate_integral);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: pid_throttle_integral
  {
    size_t item_size = sizeof(ros_message.pid_throttle_integral);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4_msgs
max_serialized_size_RoverDifferentialStatus(
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


  // Member: timestamp
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: measured_forward_speed
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: adjusted_forward_speed_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: measured_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: adjusted_yaw_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: clyaw_yaw_rate_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: measured_yaw_rate
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: adjusted_yaw_rate_setpoint
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pid_yaw_integral
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pid_yaw_rate_integral
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: pid_throttle_integral
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
    using DataType = px4_msgs::msg::RoverDifferentialStatus;
    is_plain =
      (
      offsetof(DataType, pid_throttle_integral) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RoverDifferentialStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::RoverDifferentialStatus *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RoverDifferentialStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<px4_msgs::msg::RoverDifferentialStatus *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RoverDifferentialStatus__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const px4_msgs::msg::RoverDifferentialStatus *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RoverDifferentialStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RoverDifferentialStatus(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RoverDifferentialStatus__callbacks = {
  "px4_msgs::msg",
  "RoverDifferentialStatus",
  _RoverDifferentialStatus__cdr_serialize,
  _RoverDifferentialStatus__cdr_deserialize,
  _RoverDifferentialStatus__get_serialized_size,
  _RoverDifferentialStatus__max_serialized_size
};

static rosidl_message_type_support_t _RoverDifferentialStatus__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RoverDifferentialStatus__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace px4_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_px4_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<px4_msgs::msg::RoverDifferentialStatus>()
{
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_RoverDifferentialStatus__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, px4_msgs, msg, RoverDifferentialStatus)() {
  return &px4_msgs::msg::typesupport_fastrtps_cpp::_RoverDifferentialStatus__handle;
}

#ifdef __cplusplus
}
#endif
