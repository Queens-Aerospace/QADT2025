// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_gz_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_gz_interfaces/msg/detail/joint_wrench__rosidl_typesupport_introspection_c.h"
#include "ros_gz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_gz_interfaces/msg/detail/joint_wrench__functions.h"
#include "ros_gz_interfaces/msg/detail/joint_wrench__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `body_1_name`
// Member `body_2_name`
#include "std_msgs/msg/string.h"
// Member `body_1_name`
// Member `body_2_name`
#include "std_msgs/msg/detail/string__rosidl_typesupport_introspection_c.h"
// Member `body_1_id`
// Member `body_2_id`
#include "std_msgs/msg/u_int32.h"
// Member `body_1_id`
// Member `body_2_id`
#include "std_msgs/msg/detail/u_int32__rosidl_typesupport_introspection_c.h"
// Member `body_1_wrench`
// Member `body_2_wrench`
#include "geometry_msgs/msg/wrench.h"
// Member `body_1_wrench`
// Member `body_2_wrench`
#include "geometry_msgs/msg/detail/wrench__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_gz_interfaces__msg__JointWrench__init(message_memory);
}

void ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_fini_function(void * message_memory)
{
  ros_gz_interfaces__msg__JointWrench__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__JointWrench, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_1_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__JointWrench, body_1_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_1_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__JointWrench, body_1_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_2_name",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__JointWrench, body_2_name),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_2_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__JointWrench, body_2_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_1_wrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__JointWrench, body_1_wrench),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "body_2_wrench",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__JointWrench, body_2_wrench),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_members = {
  "ros_gz_interfaces__msg",  // message namespace
  "JointWrench",  // message name
  7,  // number of fields
  sizeof(ros_gz_interfaces__msg__JointWrench),
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array,  // message members
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_init_function,  // function to initialize message memory (memory has to be allocated)
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_type_support_handle = {
  0,
  &ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_gz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, msg, JointWrench)() {
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, UInt32)();
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, String)();
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, UInt32)();
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Wrench)();
  ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_member_array[6].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Wrench)();
  if (!ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_type_support_handle.typesupport_identifier) {
    ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros_gz_interfaces__msg__JointWrench__rosidl_typesupport_introspection_c__JointWrench_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
