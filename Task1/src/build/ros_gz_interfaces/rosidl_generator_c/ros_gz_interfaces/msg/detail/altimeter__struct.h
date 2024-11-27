// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/Altimeter.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_H_

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

/// Struct defined in msg/Altimeter in the package ros_gz_interfaces.
/**
  * A message for Altimeter readings.
 */
typedef struct ros_gz_interfaces__msg__Altimeter
{
  /// Optional header data.
  std_msgs__msg__Header header;
  /// Vertical position data, in meters.
  double vertical_position;
  /// Vertical velocity data, in meters/second.
  double vertical_velocity;
  /// Vertical reference.
  double vertical_reference;
} ros_gz_interfaces__msg__Altimeter;

// Struct for a sequence of ros_gz_interfaces__msg__Altimeter.
typedef struct ros_gz_interfaces__msg__Altimeter__Sequence
{
  ros_gz_interfaces__msg__Altimeter * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__Altimeter__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ALTIMETER__STRUCT_H_
