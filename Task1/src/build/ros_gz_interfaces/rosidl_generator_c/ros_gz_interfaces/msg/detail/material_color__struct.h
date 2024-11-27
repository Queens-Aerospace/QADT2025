// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/MaterialColor.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'FIRST'.
enum
{
  ros_gz_interfaces__msg__MaterialColor__FIRST = 0
};

/// Constant 'ALL'.
enum
{
  ros_gz_interfaces__msg__MaterialColor__ALL = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'entity'
#include "ros_gz_interfaces/msg/detail/entity__struct.h"
// Member 'ambient'
// Member 'diffuse'
// Member 'specular'
// Member 'emissive'
#include "std_msgs/msg/detail/color_rgba__struct.h"

/// Struct defined in msg/MaterialColor in the package ros_gz_interfaces.
/**
  * Entities that match to apply material color: constant definition
 */
typedef struct ros_gz_interfaces__msg__MaterialColor
{
  /// Optional header data
  std_msgs__msg__Header header;
  /// Entity to change material color
  ros_gz_interfaces__msg__Entity entity;
  /// Ambient color
  std_msgs__msg__ColorRGBA ambient;
  /// Diffuse color
  std_msgs__msg__ColorRGBA diffuse;
  /// Specular color
  std_msgs__msg__ColorRGBA specular;
  /// Emissive color
  std_msgs__msg__ColorRGBA emissive;
  /// Specular exponent
  double shininess;
  /// Entities that match to apply material color
  uint8_t entity_match;
} ros_gz_interfaces__msg__MaterialColor;

// Struct for a sequence of ros_gz_interfaces__msg__MaterialColor.
typedef struct ros_gz_interfaces__msg__MaterialColor__Sequence
{
  ros_gz_interfaces__msg__MaterialColor * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__MaterialColor__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__MATERIAL_COLOR__STRUCT_H_
