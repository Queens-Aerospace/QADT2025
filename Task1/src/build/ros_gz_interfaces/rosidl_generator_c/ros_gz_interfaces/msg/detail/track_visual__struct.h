// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/TrackVisual.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_H_

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
#include "rosidl_runtime_c/string.h"
// Member 'xyz'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/TrackVisual in the package ros_gz_interfaces.
/**
  * Message for a tracking a rendering::Visual with a rendering::Camera.
 */
typedef struct ros_gz_interfaces__msg__TrackVisual
{
  /// Optional header data.
  std_msgs__msg__Header header;
  /// Name of the visual to track.
  rosidl_runtime_c__String name;
  /// Id of the visual to track.
  uint32_t id;
  /// True to have the tracking camera inherit the orientation of
  /// the tracked visual.
  bool inherit_orientation;
  /// Minimum follow distance.
  double min_dist;
  /// Maximum follow distance.
  double max_dist;
  /// If set to true, the position of the camera is fixed.
  bool is_static;
  /// If set to true, the position of the camera is relative to the.
  /// model reference frame.
  bool use_model_frame;
  /// Position of the camera.
  geometry_msgs__msg__Vector3 xyz;
  /// If set to true, the camera inherits the yaw rotation of the model.
  bool inherit_yaw;
} ros_gz_interfaces__msg__TrackVisual;

// Struct for a sequence of ros_gz_interfaces__msg__TrackVisual.
typedef struct ros_gz_interfaces__msg__TrackVisual__Sequence
{
  ros_gz_interfaces__msg__TrackVisual * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__TrackVisual__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__TRACK_VISUAL__STRUCT_H_
