// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:srv/ControlWorld.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__SRV__DETAIL__CONTROL_WORLD__STRUCT_H_
#define ROS_GZ_INTERFACES__SRV__DETAIL__CONTROL_WORLD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'world_control'
#include "ros_gz_interfaces/msg/detail/world_control__struct.h"

/// Struct defined in srv/ControlWorld in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__srv__ControlWorld_Request
{
  /// Message to Control world in Gazebo Sim
  ros_gz_interfaces__msg__WorldControl world_control;
} ros_gz_interfaces__srv__ControlWorld_Request;

// Struct for a sequence of ros_gz_interfaces__srv__ControlWorld_Request.
typedef struct ros_gz_interfaces__srv__ControlWorld_Request__Sequence
{
  ros_gz_interfaces__srv__ControlWorld_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__srv__ControlWorld_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/ControlWorld in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__srv__ControlWorld_Response
{
  /// Return true if control is successful.
  bool success;
} ros_gz_interfaces__srv__ControlWorld_Response;

// Struct for a sequence of ros_gz_interfaces__srv__ControlWorld_Response.
typedef struct ros_gz_interfaces__srv__ControlWorld_Response__Sequence
{
  ros_gz_interfaces__srv__ControlWorld_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__srv__ControlWorld_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__SRV__DETAIL__CONTROL_WORLD__STRUCT_H_
