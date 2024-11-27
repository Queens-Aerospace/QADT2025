// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/VehicleAcceleration.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__VEHICLE_ACCELERATION__STRUCT_H_
#define PX4__MSG__DETAIL__VEHICLE_ACCELERATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/VehicleAcceleration in the package px4.
typedef struct px4__msg__VehicleAcceleration
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// the timestamp of the raw data (microseconds)
  uint64_t timestamp_sample;
  /// Bias corrected acceleration (including gravity) in the FRD body frame XYZ-axis in m/s^2
  float xyz[3];
} px4__msg__VehicleAcceleration;

// Struct for a sequence of px4__msg__VehicleAcceleration.
typedef struct px4__msg__VehicleAcceleration__Sequence
{
  px4__msg__VehicleAcceleration * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__VehicleAcceleration__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__VEHICLE_ACCELERATION__STRUCT_H_
