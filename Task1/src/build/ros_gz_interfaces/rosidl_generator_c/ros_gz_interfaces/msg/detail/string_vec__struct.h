// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/StringVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__STRING_VEC__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__STRING_VEC__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/StringVec in the package ros_gz_interfaces.
/**
  * A message for a vector of string data.
 */
typedef struct ros_gz_interfaces__msg__StringVec
{
  /// Optional header data.
  std_msgs__msg__Header header;
  /// The vector of strings.
  rosidl_runtime_c__String__Sequence data;
} ros_gz_interfaces__msg__StringVec;

// Struct for a sequence of ros_gz_interfaces__msg__StringVec.
typedef struct ros_gz_interfaces__msg__StringVec__Sequence
{
  ros_gz_interfaces__msg__StringVec * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__StringVec__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__STRING_VEC__STRUCT_H_
