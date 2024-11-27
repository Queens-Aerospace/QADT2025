// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/VideoRecord.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_H_

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
// Member 'format'
// Member 'save_filename'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/VideoRecord in the package ros_gz_interfaces.
/**
  * A message that allows for control of video recording functions.
 */
typedef struct ros_gz_interfaces__msg__VideoRecord
{
  /// Optional header data.
  std_msgs__msg__Header header;
  /// True to start video recording.
  bool start;
  /// True to stop video recording.
  bool stop;
  /// Video encoding format, e.g. "mp4", "ogv".
  rosidl_runtime_c__String format;
  /// filename of the recorded video.
  rosidl_runtime_c__String save_filename;
} ros_gz_interfaces__msg__VideoRecord;

// Struct for a sequence of ros_gz_interfaces__msg__VideoRecord.
typedef struct ros_gz_interfaces__msg__VideoRecord__Sequence
{
  ros_gz_interfaces__msg__VideoRecord * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__VideoRecord__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__VIDEO_RECORD__STRUCT_H_
