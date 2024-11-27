// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/EntityWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__STRUCT_H_

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
// Member 'entity'
#include "ros_gz_interfaces/msg/detail/entity__struct.h"
// Member 'wrench'
#include "geometry_msgs/msg/detail/wrench__struct.h"

/// Struct defined in msg/EntityWrench in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__EntityWrench
{
  /// Time stamp
  std_msgs__msg__Header header;
  /// Entity
  ros_gz_interfaces__msg__Entity entity;
  /// Wrench to be applied to entity
  geometry_msgs__msg__Wrench wrench;
} ros_gz_interfaces__msg__EntityWrench;

// Struct for a sequence of ros_gz_interfaces__msg__EntityWrench.
typedef struct ros_gz_interfaces__msg__EntityWrench__Sequence
{
  ros_gz_interfaces__msg__EntityWrench * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__EntityWrench__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_WRENCH__STRUCT_H_
