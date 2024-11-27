// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/MountOrientation.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__MOUNT_ORIENTATION__STRUCT_H_
#define PX4__MSG__DETAIL__MOUNT_ORIENTATION__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/MountOrientation in the package px4.
typedef struct px4__msg__MountOrientation
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Attitude/direction of the mount as euler angles in rad
  float attitude_euler_angle[3];
} px4__msg__MountOrientation;

// Struct for a sequence of px4__msg__MountOrientation.
typedef struct px4__msg__MountOrientation__Sequence
{
  px4__msg__MountOrientation * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__MountOrientation__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__MOUNT_ORIENTATION__STRUCT_H_
