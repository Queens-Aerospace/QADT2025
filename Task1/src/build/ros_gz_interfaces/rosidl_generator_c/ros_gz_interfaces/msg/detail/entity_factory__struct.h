// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'name'
// Member 'sdf'
// Member 'sdf_filename'
// Member 'clone_name'
// Member 'relative_to'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in msg/EntityFactory in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__EntityFactory
{
  /// New name for the entity, overrides the name on the SDF
  rosidl_runtime_c__String name;
  /// Whether the server is allowed to rename the entity in case of
  /// overlap with existing entities.
  bool allow_renaming;
  /// Only one method is supported at a time (sdf,sdf_filename,clone_name)
  /// SDF description in string format
  rosidl_runtime_c__String sdf;
  /// Full path to SDF file.
  rosidl_runtime_c__String sdf_filename;
  /// Name of entity to clone
  rosidl_runtime_c__String clone_name;
  /// Pose where the entity will be spawned in the world.
  geometry_msgs__msg__Pose pose;
  /// Pose is defined relative to the frame of this entity.
  rosidl_runtime_c__String relative_to;
} ros_gz_interfaces__msg__EntityFactory;

// Struct for a sequence of ros_gz_interfaces__msg__EntityFactory.
typedef struct ros_gz_interfaces__msg__EntityFactory__Sequence
{
  ros_gz_interfaces__msg__EntityFactory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__EntityFactory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY_FACTORY__STRUCT_H_
