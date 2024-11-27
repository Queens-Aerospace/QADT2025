// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/Dataframe.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_H_

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
// Member 'src_address'
// Member 'dst_address'
#include "rosidl_runtime_c/string.h"
// Member 'data'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Dataframe in the package ros_gz_interfaces.
/**
  * Message containing some payload as well as sender and destination
  * Intended for use with the comms system in Gazebo
 */
typedef struct ros_gz_interfaces__msg__Dataframe
{
  /// Time stamp
  std_msgs__msg__Header header;
  /// Address of the sender
  rosidl_runtime_c__String src_address;
  /// Address of the destination
  rosidl_runtime_c__String dst_address;
  /// Payload
  rosidl_runtime_c__uint8__Sequence data;
  /// Received Signal Strength Indicator
  double rssi;
} ros_gz_interfaces__msg__Dataframe;

// Struct for a sequence of ros_gz_interfaces__msg__Dataframe.
typedef struct ros_gz_interfaces__msg__Dataframe__Sequence
{
  ros_gz_interfaces__msg__Dataframe * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__Dataframe__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__DATAFRAME__STRUCT_H_
