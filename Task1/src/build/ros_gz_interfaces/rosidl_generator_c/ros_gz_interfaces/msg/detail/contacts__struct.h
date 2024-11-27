// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/Contacts.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__CONTACTS__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__CONTACTS__STRUCT_H_

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
// Member 'contacts'
#include "ros_gz_interfaces/msg/detail/contact__struct.h"

/// Struct defined in msg/Contacts in the package ros_gz_interfaces.
typedef struct ros_gz_interfaces__msg__Contacts
{
  /// Time stamp
  std_msgs__msg__Header header;
  /// List of contacts
  ros_gz_interfaces__msg__Contact__Sequence contacts;
} ros_gz_interfaces__msg__Contacts;

// Struct for a sequence of ros_gz_interfaces__msg__Contacts.
typedef struct ros_gz_interfaces__msg__Contacts__Sequence
{
  ros_gz_interfaces__msg__Contacts * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__Contacts__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__CONTACTS__STRUCT_H_
