// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverMecanumStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverMecanumStatus in the package px4_msgs.
typedef struct px4_msgs__msg__RoverMecanumStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Measured speed in body x direction. Positiv: forwards, Negativ: backwards
  float measured_forward_speed;
  /// Measured speed in body y direction. Positiv: right, Negativ: left
  float measured_lateral_speed;
  /// Yaw rate setpoint output of the closed loop yaw controller
  float adjusted_yaw_rate_setpoint;
  /// Measured yaw rate
  float measured_yaw_rate;
  /// Measured yaw
  float measured_yaw;
  /// Integral of the PID for the closed loop yaw rate controller
  float pid_yaw_rate_integral;
  /// Integral of the PID for the closed loop yaw controller
  float pid_yaw_integral;
  /// Integral of the PID for the closed loop forward speed controller
  float pid_forward_throttle_integral;
  /// Integral of the PID for the closed loop lateral speed controller
  float pid_lateral_throttle_integral;
} px4_msgs__msg__RoverMecanumStatus;

// Struct for a sequence of px4_msgs__msg__RoverMecanumStatus.
typedef struct px4_msgs__msg__RoverMecanumStatus__Sequence
{
  px4_msgs__msg__RoverMecanumStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverMecanumStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_MECANUM_STATUS__STRUCT_H_
