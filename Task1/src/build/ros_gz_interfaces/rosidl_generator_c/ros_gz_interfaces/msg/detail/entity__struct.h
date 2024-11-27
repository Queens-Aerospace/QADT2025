// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/Entity.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NONE'.
enum
{
  ros_gz_interfaces__msg__Entity__NONE = 0
};

/// Constant 'LIGHT'.
enum
{
  ros_gz_interfaces__msg__Entity__LIGHT = 1
};

/// Constant 'MODEL'.
enum
{
  ros_gz_interfaces__msg__Entity__MODEL = 2
};

/// Constant 'LINK'.
enum
{
  ros_gz_interfaces__msg__Entity__LINK = 3
};

/// Constant 'VISUAL'.
enum
{
  ros_gz_interfaces__msg__Entity__VISUAL = 4
};

/// Constant 'COLLISION'.
enum
{
  ros_gz_interfaces__msg__Entity__COLLISION = 5
};

/// Constant 'SENSOR'.
enum
{
  ros_gz_interfaces__msg__Entity__SENSOR = 6
};

/// Constant 'JOINT'.
enum
{
  ros_gz_interfaces__msg__Entity__JOINT = 7
};

// Include directives for member types
// Member 'name'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/Entity in the package ros_gz_interfaces.
/**
  * Entity type: constant definition
 */
typedef struct ros_gz_interfaces__msg__Entity
{
  /// Entity unique identifier accross all types. Defaults to 0
  uint64_t id;
  /// Entity name, which is not guaranteed to be unique.
  rosidl_runtime_c__String name;
  /// Entity type.
  uint8_t type;
} ros_gz_interfaces__msg__Entity;

// Struct for a sequence of ros_gz_interfaces__msg__Entity.
typedef struct ros_gz_interfaces__msg__Entity__Sequence
{
  ros_gz_interfaces__msg__Entity * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__Entity__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__ENTITY__STRUCT_H_
