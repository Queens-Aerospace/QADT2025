// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/MavlinkLog.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MAVLINK_LOG__STRUCT_H_
#define PX4__MSG__DETAIL__MAVLINK_LOG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ORB_QUEUE_LENGTH'.
enum
{
  px4__msg__MavlinkLog__ORB_QUEUE_LENGTH = 8
};

/// Struct defined in msg/MavlinkLog in the package px4.
typedef struct px4__msg__MavlinkLog
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  uint8_t text[127];
  /// log level (same as in the linux kernel, starting with 0)
  uint8_t severity;
} px4__msg__MavlinkLog;

// Struct for a sequence of px4__msg__MavlinkLog.
typedef struct px4__msg__MavlinkLog__Sequence
{
  px4__msg__MavlinkLog * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__MavlinkLog__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__MAVLINK_LOG__STRUCT_H_
