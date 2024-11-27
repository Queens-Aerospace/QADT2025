// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from px4:msg/VtolVehicleStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__VTOL_VEHICLE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define PX4__MSG__DETAIL__VTOL_VEHICLE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "px4/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "px4/msg/detail/vtol_vehicle_status__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace px4
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4
cdr_serialize(
  const px4::msg::VtolVehicleStatus & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  px4::msg::VtolVehicleStatus & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4
get_serialized_size(
  const px4::msg::VtolVehicleStatus & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4
max_serialized_size_VtolVehicleStatus(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace px4

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_px4
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, px4, msg, VtolVehicleStatus)();

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__VTOL_VEHICLE_STATUS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
