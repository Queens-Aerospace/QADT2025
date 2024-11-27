// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/PositionControllerStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__POSITION_CONTROLLER_STATUS__STRUCT_H_
#define PX4__MSG__DETAIL__POSITION_CONTROLLER_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PositionControllerStatus in the package px4.
typedef struct px4__msg__PositionControllerStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Roll setpoint
  float nav_roll;
  /// Pitch setpoint
  float nav_pitch;
  /// Bearing angle
  float nav_bearing;
  /// Bearing angle from aircraft to current target
  float target_bearing;
  /// Signed track error
  float xtrack_error;
  /// Distance to active (next) waypoint
  float wp_dist;
  /// Current horizontal acceptance radius
  float acceptance_radius;
  /// Yaw acceptance error
  float yaw_acceptance;
  /// Current vertical acceptance error
  float altitude_acceptance;
  /// Current (applied) position setpoint type (see PositionSetpoint.msg)
  uint8_t type;
} px4__msg__PositionControllerStatus;

// Struct for a sequence of px4__msg__PositionControllerStatus.
typedef struct px4__msg__PositionControllerStatus__Sequence
{
  px4__msg__PositionControllerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__PositionControllerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__POSITION_CONTROLLER_STATUS__STRUCT_H_
