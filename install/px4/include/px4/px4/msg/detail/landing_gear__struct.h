// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/LandingGear.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__LANDING_GEAR__STRUCT_H_
#define PX4__MSG__DETAIL__LANDING_GEAR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'GEAR_UP'.
/**
  * landing gear up
 */
enum
{
  px4__msg__LandingGear__GEAR_UP = 1
};

/// Constant 'GEAR_DOWN'.
/**
  * landing gear down
 */
enum
{
  px4__msg__LandingGear__GEAR_DOWN = -1
};

/// Constant 'GEAR_KEEP'.
/**
  * keep the current state
 */
enum
{
  px4__msg__LandingGear__GEAR_KEEP = 0
};

/// Struct defined in msg/LandingGear in the package px4.
typedef struct px4__msg__LandingGear
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  int8_t landing_gear;
} px4__msg__LandingGear;

// Struct for a sequence of px4__msg__LandingGear.
typedef struct px4__msg__LandingGear__Sequence
{
  px4__msg__LandingGear * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__LandingGear__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__LANDING_GEAR__STRUCT_H_
