// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/DebugVect.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__DEBUG_VECT__STRUCT_H_
#define PX4__MSG__DETAIL__DEBUG_VECT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/DebugVect in the package px4.
typedef struct px4__msg__DebugVect
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// max. 10 characters as key / name
  uint8_t name[10];
  /// x value
  float x;
  /// y value
  float y;
  /// z value
  float z;
} px4__msg__DebugVect;

// Struct for a sequence of px4__msg__DebugVect.
typedef struct px4__msg__DebugVect__Sequence
{
  px4__msg__DebugVect * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__DebugVect__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__DEBUG_VECT__STRUCT_H_
