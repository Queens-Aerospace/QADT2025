// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/ModeCompleted.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MODE_COMPLETED__STRUCT_H_
#define PX4__MSG__DETAIL__MODE_COMPLETED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'RESULT_SUCCESS'.
enum
{
  px4__msg__ModeCompleted__RESULT_SUCCESS = 0
};

/// Constant 'RESULT_FAILURE_OTHER'.
/**
  * : reserved
  *  Mode failed (generic error)
 */
enum
{
  px4__msg__ModeCompleted__RESULT_FAILURE_OTHER = 100
};

/// Struct defined in msg/ModeCompleted in the package px4.
/**
  * Mode completion result, published by an active mode.
  * The possible values of nav_state are defined in the VehicleStatus msg.
  * Note that this is not always published (e.g. when a user switches modes or on
  * failsafe activation)
 */
typedef struct px4__msg__ModeCompleted
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// One of RESULT_*
  uint8_t result;
  /// Source mode (values in VehicleStatus)
  uint8_t nav_state;
} px4__msg__ModeCompleted;

// Struct for a sequence of px4__msg__ModeCompleted.
typedef struct px4__msg__ModeCompleted__Sequence
{
  px4__msg__ModeCompleted * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__ModeCompleted__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__MODE_COMPLETED__STRUCT_H_
