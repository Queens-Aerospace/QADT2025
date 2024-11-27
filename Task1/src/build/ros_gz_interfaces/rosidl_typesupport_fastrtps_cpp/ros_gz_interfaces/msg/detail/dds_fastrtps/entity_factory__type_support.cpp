// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ros_gz_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/entity_factory__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ros_gz_interfaces/msg/detail/entity_factory__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace geometry_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const geometry_msgs::msg::Pose &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  geometry_msgs::msg::Pose &);
size_t get_serialized_size(
  const geometry_msgs::msg::Pose &,
  size_t current_alignment);
size_t
max_serialized_size_Pose(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace geometry_msgs


namespace ros_gz_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
cdr_serialize(
  const ros_gz_interfaces::msg::EntityFactory & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: name
  cdr << ros_message.name;
  // Member: allow_renaming
  cdr << (ros_message.allow_renaming ? true : false);
  // Member: sdf
  cdr << ros_message.sdf;
  // Member: sdf_filename
  cdr << ros_message.sdf_filename;
  // Member: clone_name
  cdr << ros_message.clone_name;
  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.pose,
    cdr);
  // Member: relative_to
  cdr << ros_message.relative_to;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ros_gz_interfaces::msg::EntityFactory & ros_message)
{
  // Member: name
  cdr >> ros_message.name;

  // Member: allow_renaming
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.allow_renaming = tmp ? true : false;
  }

  // Member: sdf
  cdr >> ros_message.sdf;

  // Member: sdf_filename
  cdr >> ros_message.sdf_filename;

  // Member: clone_name
  cdr >> ros_message.clone_name;

  // Member: pose
  geometry_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.pose);

  // Member: relative_to
  cdr >> ros_message.relative_to;

  return true;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
get_serialized_size(
  const ros_gz_interfaces::msg::EntityFactory & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.name.size() + 1);
  // Member: allow_renaming
  {
    size_t item_size = sizeof(ros_message.allow_renaming);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: sdf
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.sdf.size() + 1);
  // Member: sdf_filename
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.sdf_filename.size() + 1);
  // Member: clone_name
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.clone_name.size() + 1);
  // Member: pose

  current_alignment +=
    geometry_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.pose, current_alignment);
  // Member: relative_to
  current_alignment += padding +
    eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
    (ros_message.relative_to.size() + 1);

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ros_gz_interfaces
max_serialized_size_EntityFactory(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: allow_renaming
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: sdf
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: sdf_filename
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: clone_name
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  // Member: pose
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        geometry_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Pose(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: relative_to
  {
    size_t array_size = 1;

    full_bounded = false;
    is_plain = false;
    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += padding +
        eprosima::fastcdr::Cdr::alignment(current_alignment, padding) +
        1;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = ros_gz_interfaces::msg::EntityFactory;
    is_plain =
      (
      offsetof(DataType, relative_to) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _EntityFactory__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ros_gz_interfaces::msg::EntityFactory *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _EntityFactory__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ros_gz_interfaces::msg::EntityFactory *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _EntityFactory__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ros_gz_interfaces::msg::EntityFactory *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _EntityFactory__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_EntityFactory(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _EntityFactory__callbacks = {
  "ros_gz_interfaces::msg",
  "EntityFactory",
  _EntityFactory__cdr_serialize,
  _EntityFactory__cdr_deserialize,
  _EntityFactory__get_serialized_size,
  _EntityFactory__max_serialized_size
};

static rosidl_message_type_support_t _EntityFactory__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_EntityFactory__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ros_gz_interfaces

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ros_gz_interfaces
const rosidl_message_type_support_t *
get_message_type_support_handle<ros_gz_interfaces::msg::EntityFactory>()
{
  return &ros_gz_interfaces::msg::typesupport_fastrtps_cpp::_EntityFactory__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ros_gz_interfaces, msg, EntityFactory)() {
  return &ros_gz_interfaces::msg::typesupport_fastrtps_cpp::_EntityFactory__handle;
}

#ifdef __cplusplus
}
#endif
