// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/GuiCamera.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__STRUCT_H_

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
// Member 'name'
// Member 'view_controller'
// Member 'projection_type'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'track'
#include "ros_gz_interfaces/msg/detail/track_visual__struct.h"

/// Struct defined in msg/GuiCamera in the package ros_gz_interfaces.
/**
  * Message for a GUI Camera.
 */
typedef struct ros_gz_interfaces__msg__GuiCamera
{
  /// Optional header data.
  std_msgs__msg__Header header;
  rosidl_runtime_c__String name;
  rosidl_runtime_c__String view_controller;
  geometry_msgs__msg__Pose pose;
  ros_gz_interfaces__msg__TrackVisual track;
  /// Type of projection: "perspective" or "orthographic".
  rosidl_runtime_c__String projection_type;
} ros_gz_interfaces__msg__GuiCamera;

// Struct for a sequence of ros_gz_interfaces__msg__GuiCamera.
typedef struct ros_gz_interfaces__msg__GuiCamera__Sequence
{
  ros_gz_interfaces__msg__GuiCamera * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__GuiCamera__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__GUI_CAMERA__STRUCT_H_
