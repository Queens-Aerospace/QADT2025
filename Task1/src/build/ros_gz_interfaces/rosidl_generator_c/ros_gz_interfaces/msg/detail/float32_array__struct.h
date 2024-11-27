// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/Float32Array.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Float32Array in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__Float32Array
{
  rosidl_runtime_c__float__Sequence data;
} ros_gz_interfaces__msg__Float32Array;

// Struct for a sequence of ros_gz_interfaces__msg__Float32Array.
typedef struct ros_gz_interfaces__msg__Float32Array__Sequence
{
  ros_gz_interfaces__msg__Float32Array * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__Float32Array__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__FLOAT32_ARRAY__STRUCT_H_
