// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from ros_gz_interfaces:msg/Contacts.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "ros_gz_interfaces/msg/detail/contacts__rosidl_typesupport_introspection_c.h"
#include "ros_gz_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "ros_gz_interfaces/msg/detail/contacts__functions.h"
#include "ros_gz_interfaces/msg/detail/contacts__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `contacts`
#include "ros_gz_interfaces/msg/contact.h"
// Member `contacts`
#include "ros_gz_interfaces/msg/detail/contact__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  ros_gz_interfaces__msg__Contacts__init(message_memory);
}

void ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_fini_function(void * message_memory)
{
  ros_gz_interfaces__msg__Contacts__fini(message_memory);
}

size_t ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__size_function__Contacts__contacts(
  const void * untyped_member)
{
  const ros_gz_interfaces__msg__Contact__Sequence * member =
    (const ros_gz_interfaces__msg__Contact__Sequence *)(untyped_member);
  return member->size;
}

const void * ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__get_const_function__Contacts__contacts(
  const void * untyped_member, size_t index)
{
  const ros_gz_interfaces__msg__Contact__Sequence * member =
    (const ros_gz_interfaces__msg__Contact__Sequence *)(untyped_member);
  return &member->data[index];
}

void * ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__get_function__Contacts__contacts(
  void * untyped_member, size_t index)
{
  ros_gz_interfaces__msg__Contact__Sequence * member =
    (ros_gz_interfaces__msg__Contact__Sequence *)(untyped_member);
  return &member->data[index];
}

void ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__fetch_function__Contacts__contacts(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const ros_gz_interfaces__msg__Contact * item =
    ((const ros_gz_interfaces__msg__Contact *)
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__get_const_function__Contacts__contacts(untyped_member, index));
  ros_gz_interfaces__msg__Contact * value =
    (ros_gz_interfaces__msg__Contact *)(untyped_value);
  *value = *item;
}

void ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__assign_function__Contacts__contacts(
  void * untyped_member, size_t index, const void * untyped_value)
{
  ros_gz_interfaces__msg__Contact * item =
    ((ros_gz_interfaces__msg__Contact *)
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__get_function__Contacts__contacts(untyped_member, index));
  const ros_gz_interfaces__msg__Contact * value =
    (const ros_gz_interfaces__msg__Contact *)(untyped_value);
  *item = *value;
}

bool ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__resize_function__Contacts__contacts(
  void * untyped_member, size_t size)
{
  ros_gz_interfaces__msg__Contact__Sequence * member =
    (ros_gz_interfaces__msg__Contact__Sequence *)(untyped_member);
  ros_gz_interfaces__msg__Contact__Sequence__fini(member);
  return ros_gz_interfaces__msg__Contact__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_member_array[2] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__Contacts, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "contacts",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(ros_gz_interfaces__msg__Contacts, contacts),  // bytes offset in struct
    NULL,  // default value
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__size_function__Contacts__contacts,  // size() function pointer
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__get_const_function__Contacts__contacts,  // get_const(index) function pointer
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__get_function__Contacts__contacts,  // get(index) function pointer
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__fetch_function__Contacts__contacts,  // fetch(index, &value) function pointer
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__assign_function__Contacts__contacts,  // assign(index, value) function pointer
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__resize_function__Contacts__contacts  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_members = {
  "ros_gz_interfaces__msg",  // message namespace
  "Contacts",  // message name
  2,  // number of fields
  sizeof(ros_gz_interfaces__msg__Contacts),
  ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_member_array,  // message members
  ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_init_function,  // function to initialize message memory (memory has to be allocated)
  ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_type_support_handle = {
  0,
  &ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_ros_gz_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, msg, Contacts)() {
  ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, ros_gz_interfaces, msg, Contact)();
  if (!ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_type_support_handle.typesupport_identifier) {
    ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &ros_gz_interfaces__msg__Contacts__rosidl_typesupport_introspection_c__Contacts_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
