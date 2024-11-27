// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverAckermannStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverAckermannStatus in the package px4_msgs.
typedef struct px4_msgs__msg__RoverAckermannStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Measured speed in body x direction. Forwards: positiv, Backwards: negativ
  float measured_forward_speed;
  /// Speed setpoint after applying slew rate
  float adjusted_forward_speed_setpoint;
  /// [-1, 1] Normalized steering setpoint
  float steering_setpoint_normalized;
  /// [-1, 1] Normalized steering setpoint after applying slew rate
  float adjusted_steering_setpoint_normalized;
  /// Measured acceleration in body y direction. Positiv: right, Negativ: left.
  float measured_lateral_acceleration;
  /// Integral of the PID for the closed loop speed controller
  float pid_throttle_integral;
  /// Integral of the PID for the closed loop lateral acceleration controller
  float pid_lat_accel_integral;
} px4_msgs__msg__RoverAckermannStatus;

// Struct for a sequence of px4_msgs__msg__RoverAckermannStatus.
typedef struct px4_msgs__msg__RoverAckermannStatus__Sequence
{
  px4_msgs__msg__RoverAckermannStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverAckermannStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_ACKERMANN_STATUS__STRUCT_H_
