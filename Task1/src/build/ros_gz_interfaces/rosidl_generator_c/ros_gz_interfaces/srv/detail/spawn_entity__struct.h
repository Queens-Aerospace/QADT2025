// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:srv/SpawnEntity.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__SRV__DETAIL__SPAWN_ENTITY__STRUCT_H_
#define ROS_GZ_INTERFACES__SRV__DETAIL__SPAWN_ENTITY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'entity_factory'
#include "ros_gz_interfaces/msg/detail/entity_factory__struct.h"

/// Struct defined in srv/SpawnEntity in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__srv__SpawnEntity_Request
{
  /// Message to create a new entity
  ros_gz_interfaces__msg__EntityFactory entity_factory;
} ros_gz_interfaces__srv__SpawnEntity_Request;

// Struct for a sequence of ros_gz_interfaces__srv__SpawnEntity_Request.
typedef struct ros_gz_interfaces__srv__SpawnEntity_Request__Sequence
{
  ros_gz_interfaces__srv__SpawnEntity_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__srv__SpawnEntity_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/SpawnEntity in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__srv__SpawnEntity_Response
{
  /// Return true if spawned successfully.
  bool success;
} ros_gz_interfaces__srv__SpawnEntity_Response;

// Struct for a sequence of ros_gz_interfaces__srv__SpawnEntity_Response.
typedef struct ros_gz_interfaces__srv__SpawnEntity_Response__Sequence
{
  ros_gz_interfaces__srv__SpawnEntity_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__srv__SpawnEntity_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__SRV__DETAIL__SPAWN_ENTITY__STRUCT_H_
