// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/ParamVec.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_H_

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
// Member 'params'
#include "rcl_interfaces/msg/detail/parameter__struct.h"

/// Struct defined in msg/ParamVec in the package ros_gz_interfaces.
/**
  * A message for a vector of parameters data.
 */
typedef struct ros_gz_interfaces__msg__ParamVec
{
  /// Optional header data.
  std_msgs__msg__Header header;
  /// A set of name, value pairs
  rcl_interfaces__msg__Parameter__Sequence params;
} ros_gz_interfaces__msg__ParamVec;

// Struct for a sequence of ros_gz_interfaces__msg__ParamVec.
typedef struct ros_gz_interfaces__msg__ParamVec__Sequence
{
  ros_gz_interfaces__msg__ParamVec * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__ParamVec__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__PARAM_VEC__STRUCT_H_
