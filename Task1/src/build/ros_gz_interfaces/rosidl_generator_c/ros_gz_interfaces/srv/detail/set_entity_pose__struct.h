// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:srv/SetEntityPose.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__SRV__DETAIL__SET_ENTITY_POSE__STRUCT_H_
#define ROS_GZ_INTERFACES__SRV__DETAIL__SET_ENTITY_POSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'entity'
#include "ros_gz_interfaces/msg/detail/entity__struct.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/SetEntityPose in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__srv__SetEntityPose_Request
{
  /// Gazebo Sim entity.
  ros_gz_interfaces__msg__Entity entity;
  /// Pose of entity.
  geometry_msgs__msg__Pose pose;
} ros_gz_interfaces__srv__SetEntityPose_Request;

// Struct for a sequence of ros_gz_interfaces__srv__SetEntityPose_Request.
typedef struct ros_gz_interfaces__srv__SetEntityPose_Request__Sequence
{
  ros_gz_interfaces__srv__SetEntityPose_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__srv__SetEntityPose_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SetEntityPose in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__srv__SetEntityPose_Response
{
  /// Return true if set successfully.
  bool success;
} ros_gz_interfaces__srv__SetEntityPose_Response;

// Struct for a sequence of ros_gz_interfaces__srv__SetEntityPose_Response.
typedef struct ros_gz_interfaces__srv__SetEntityPose_Response__Sequence
{
  ros_gz_interfaces__srv__SetEntityPose_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__srv__SetEntityPose_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__SRV__DETAIL__SET_ENTITY_POSE__STRUCT_H_
