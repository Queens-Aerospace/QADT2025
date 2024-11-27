// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__STRUCT_H_

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
// Member 'body_1_name'
// Member 'body_2_name'
#include "std_msgs/msg/detail/string__struct.h"
// Member 'body_1_id'
// Member 'body_2_id'
#include "std_msgs/msg/detail/u_int32__struct.h"
// Member 'body_1_wrench'
// Member 'body_2_wrench'
#include "geometry_msgs/msg/detail/wrench__struct.h"

/// Struct defined in msg/JointWrench in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__JointWrench
{
  /// Time stamp
  std_msgs__msg__Header header;
  /// Body 1 name string
  std_msgs__msg__String body_1_name;
  /// Body 1 id
  std_msgs__msg__UInt32 body_1_id;
  /// Body 2 name string
  std_msgs__msg__String body_2_name;
  /// Body 2 id
  std_msgs__msg__UInt32 body_2_id;
  /// Body 1 wrench
  geometry_msgs__msg__Wrench body_1_wrench;
  /// Body 2 wrench
  geometry_msgs__msg__Wrench body_2_wrench;
} ros_gz_interfaces__msg__JointWrench;

// Struct for a sequence of ros_gz_interfaces__msg__JointWrench.
typedef struct ros_gz_interfaces__msg__JointWrench__Sequence
{
  ros_gz_interfaces__msg__JointWrench * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__JointWrench__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__JOINT_WRENCH__STRUCT_H_
