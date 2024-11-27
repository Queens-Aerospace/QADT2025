// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/LandingTargetInnovations.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__LANDING_TARGET_INNOVATIONS__STRUCT_H_
#define PX4__MSG__DETAIL__LANDING_TARGET_INNOVATIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/LandingTargetInnovations in the package px4.
typedef struct px4__msg__LandingTargetInnovations
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  /// Innovation of landing target position estimator
  float innov_x;
  float innov_y;
  /// Innovation covariance of landing target position estimator
  float innov_cov_x;
  float innov_cov_y;
} px4__msg__LandingTargetInnovations;

// Struct for a sequence of px4__msg__LandingTargetInnovations.
typedef struct px4__msg__LandingTargetInnovations__Sequence
{
  px4__msg__LandingTargetInnovations * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__LandingTargetInnovations__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__LANDING_TARGET_INNOVATIONS__STRUCT_H_
