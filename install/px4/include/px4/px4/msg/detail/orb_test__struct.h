// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/OrbTest.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ORB_TEST__STRUCT_H_
#define PX4__MSG__DETAIL__ORB_TEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/OrbTest in the package px4.
typedef struct px4__msg__OrbTest
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  int32_t val;
} px4__msg__OrbTest;

// Struct for a sequence of px4__msg__OrbTest.
typedef struct px4__msg__OrbTest__Sequence
{
  px4__msg__OrbTest * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__OrbTest__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__ORB_TEST__STRUCT_H_
