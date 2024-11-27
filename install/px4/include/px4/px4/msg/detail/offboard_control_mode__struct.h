// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/OffboardControlMode.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__OFFBOARD_CONTROL_MODE__STRUCT_H_
#define PX4__MSG__DETAIL__OFFBOARD_CONTROL_MODE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/OffboardControlMode in the package px4.
/**
  * Off-board control mode
 */
typedef struct px4__msg__OffboardControlMode
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  bool position;
  bool velocity;
  bool acceleration;
  bool attitude;
  bool body_rate;
  bool thrust_and_torque;
  bool direct_actuator;
} px4__msg__OffboardControlMode;

// Struct for a sequence of px4__msg__OffboardControlMode.
typedef struct px4__msg__OffboardControlMode__Sequence
{
  px4__msg__OffboardControlMode * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__OffboardControlMode__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__OFFBOARD_CONTROL_MODE__STRUCT_H_
