// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/Light.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'POINT'.
/**
  * Light type: constant definition
 */
enum
{
  ros_gz_interfaces__msg__Light__POINT = 0
};

/// Constant 'SPOT'.
enum
{
  ros_gz_interfaces__msg__Light__SPOT = 1
};

/// Constant 'DIRECTIONAL'.
enum
{
  ros_gz_interfaces__msg__Light__DIRECTIONAL = 2
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'name'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"
// Member 'diffuse'
// Member 'specular'
#include "std_msgs/msg/detail/color_rgba__struct.h"
// Member 'direction'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/Light in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__Light
{
  /// Optional header data
  std_msgs__msg__Header header;
  /// Light name
  rosidl_runtime_c__String name;
  /// Light type (from constant definitions)
  uint8_t type;
  /// Light pose
  geometry_msgs__msg__Pose pose;
  /// Light diffuse emission
  std_msgs__msg__ColorRGBA diffuse;
  /// Light specular emission
  std_msgs__msg__ColorRGBA specular;
  /// Constant variable in attenuation formula
  float attenuation_constant;
  /// Linear variable in attenuation formula
  float attenuation_linear;
  /// Quadratic variable in attenuation formula
  float attenuation_quadratic;
  /// Light direction
  geometry_msgs__msg__Vector3 direction;
  /// Light range
  float range;
  /// Enable/disable shadow casting
  bool cast_shadows;
  /// Spotlight inner cone angle
  float spot_inner_angle;
  /// Spotlight outer cone angle
  float spot_outer_angle;
  /// Falloff between inner and outer cone
  float spot_falloff;
  /// Unique id of the light
  uint32_t id;
  /// Unique id of the light's parent
  uint32_t parent_id;
  /// Light intensity
  float intensity;
} ros_gz_interfaces__msg__Light;

// Struct for a sequence of ros_gz_interfaces__msg__Light.
typedef struct ros_gz_interfaces__msg__Light__Sequence
{
  ros_gz_interfaces__msg__Light * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__Light__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__LIGHT__STRUCT_H_
