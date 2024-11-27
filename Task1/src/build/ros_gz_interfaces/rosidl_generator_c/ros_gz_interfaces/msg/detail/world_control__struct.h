// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/WorldControl.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'reset'
#include "ros_gz_interfaces/msg/detail/world_reset__struct.h"
// Member 'run_to_sim_time'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in msg/WorldControl in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__WorldControl
{
  /// Paused state.
  bool pause;
  bool step;
  /// Paused after stepping multi_step.
  uint32_t multi_step;
  ros_gz_interfaces__msg__WorldReset reset;
  uint32_t seed;
  /// A simulation time in the future to run to and
  /// then pause.
  builtin_interfaces__msg__Time run_to_sim_time;
} ros_gz_interfaces__msg__WorldControl;

// Struct for a sequence of ros_gz_interfaces__msg__WorldControl.
typedef struct ros_gz_interfaces__msg__WorldControl__Sequence
{
  ros_gz_interfaces__msg__WorldControl * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__WorldControl__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__STRUCT_H_
