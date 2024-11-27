// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_gz_interfaces:srv/SetEntityPose.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_gz_interfaces/srv/detail/set_entity_pose__rosidl_typesupport_introspection_c.h"
#include "ros_gz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_gz_interfaces/srv/detail/set_entity_pose__functions.h"
#include "ros_gz_interfaces/srv/detail/set_entity_pose__struct.h"


// Include directives for member types
// Member `entity`
#include "ros_gz_interfaces/msg/entity.h"
// Member `entity`
#include "ros_gz_interfaces/msg/detail/entity__rosidl_typesupport_introspection_c.h"
// Member `pose`
#include "geometry_msgs/msg/pose.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_gz_interfaces__srv__SetEntityPose_Request__init(message_memory);
}

void ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_fini_function(void * message_memory)
{
  ros_gz_interfaces__srv__SetEntityPose_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_member_array[2] = {
  {
    "entity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__srv__SetEntityPose_Request, entity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pose",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__srv__SetEntityPose_Request, pose),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_members = {
  "ros_gz_interfaces__srv",  // message namespace
  "SetEntityPose_Request",  // message name
  2,  // number of fields
  sizeof(ros_gz_interfaces__srv__SetEntityPose_Request),
  ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_member_array,  // message members
  ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_type_support_handle = {
  0,
  &ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_gz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, srv, SetEntityPose_Request)() {
  ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, msg, Entity)();
  ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_type_support_handle.typesupport_identifier) {
    ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros_gz_interfaces__srv__SetEntityPose_Request__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "ros_gz_interfaces/srv/detail/set_entity_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "ros_gz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "ros_gz_interfaces/srv/detail/set_entity_pose__functions.h"
// already included above
// #include "ros_gz_interfaces/srv/detail/set_entity_pose__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_gz_interfaces__srv__SetEntityPose_Response__init(message_memory);
}

void ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_fini_function(void * message_memory)
{
  ros_gz_interfaces__srv__SetEntityPose_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_member_array[1] = {
  {
    "success",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__srv__SetEntityPose_Response, success),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_members = {
  "ros_gz_interfaces__srv",  // message namespace
  "SetEntityPose_Response",  // message name
  1,  // number of fields
  sizeof(ros_gz_interfaces__srv__SetEntityPose_Response),
  ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_member_array,  // message members
  ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_type_support_handle = {
  0,
  &ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_gz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, srv, SetEntityPose_Response)() {
  if (!ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_type_support_handle.typesupport_identifier) {
    ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros_gz_interfaces__srv__SetEntityPose_Response__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "ros_gz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "ros_gz_interfaces/srv/detail/set_entity_pose__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_service_members = {
  "ros_gz_interfaces__srv",  // service namespace
  "SetEntityPose",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_Request_message_type_support_handle,
  NULL  // response message
  // ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_Response_message_type_support_handle
};

static rosidl_service_type_support_t ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_service_type_support_handle = {
  0,
  &ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, srv, SetEntityPose_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, srv, SetEntityPose_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_gz_interfaces
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, srv, SetEntityPose)() {
  if (!ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_service_type_support_handle.typesupport_identifier) {
    ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, srv, SetEntityPose_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, srv, SetEntityPose_Response)()->data;
  }

  return &ros_gz_interfaces__srv__detail__set_entity_pose__rosidl_typesupport_introspection_c__SetEntityPose_service_type_support_handle;
}
