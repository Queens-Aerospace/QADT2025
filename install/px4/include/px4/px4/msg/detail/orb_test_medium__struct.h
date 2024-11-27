// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/OrbTestMedium.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ORB_TEST_MEDIUM__STRUCT_H_
#define PX4__MSG__DETAIL__ORB_TEST_MEDIUM__STRUCT_H_

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
  px4__msg__OrbTestMedium__ORB_QUEUE_LENGTH = 16
};

/// Struct defined in msg/OrbTestMedium in the package px4.
typedef struct px4__msg__OrbTestMedium
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  int32_t val;
  uint8_t junk[64];
} px4__msg__OrbTestMedium;

// Struct for a sequence of px4__msg__OrbTestMedium.
typedef struct px4__msg__OrbTestMedium__Sequence
{
  px4__msg__OrbTestMedium * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__OrbTestMedium__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__ORB_TEST_MEDIUM__STRUCT_H_
