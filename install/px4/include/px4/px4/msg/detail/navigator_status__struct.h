// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/NavigatorStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__NAVIGATOR_STATUS__STRUCT_H_
#define PX4__MSG__DETAIL__NAVIGATOR_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FAILURE_NONE'.
enum
{
  px4__msg__NavigatorStatus__FAILURE_NONE = 0
};

/// Constant 'FAILURE_HAGL'.
/**
  * Target altitude exceeds maximum height above ground
 */
enum
{
  px4__msg__NavigatorStatus__FAILURE_HAGL = 1
};

/// Struct defined in msg/NavigatorStatus in the package px4.
/**
  * Current status of a Navigator mode
  * The possible values of nav_state are defined in the VehicleStatus msg.
 */
typedef struct px4__msg__NavigatorStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Source mode (values in VehicleStatus)
  uint8_t nav_state;
  /// Navigator failure enum
  uint8_t failure;
} px4__msg__NavigatorStatus;

// Struct for a sequence of px4__msg__NavigatorStatus.
typedef struct px4__msg__NavigatorStatus__Sequence
{
  px4__msg__NavigatorStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__NavigatorStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__NAVIGATOR_STATUS__STRUCT_H_
