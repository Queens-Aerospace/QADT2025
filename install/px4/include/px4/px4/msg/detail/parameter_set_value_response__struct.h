// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/ParameterSetValueResponse.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__PARAMETER_SET_VALUE_RESPONSE__STRUCT_H_
#define PX4__MSG__DETAIL__PARAMETER_SET_VALUE_RESPONSE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ORB_QUEUE_LENGTH'.
enum
{
  px4__msg__ParameterSetValueResponse__ORB_QUEUE_LENGTH = 4
};

/// Struct defined in msg/ParameterSetValueResponse in the package px4.
/**
  * ParameterSetValueResponse : Response to a set value request by either primary or secondary
 */
typedef struct px4__msg__ParameterSetValueResponse
{
  uint64_t timestamp;
  uint64_t request_timestamp;
  uint16_t parameter_index;
} px4__msg__ParameterSetValueResponse;

// Struct for a sequence of px4__msg__ParameterSetValueResponse.
typedef struct px4__msg__ParameterSetValueResponse__Sequence
{
  px4__msg__ParameterSetValueResponse * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__ParameterSetValueResponse__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__PARAMETER_SET_VALUE_RESPONSE__STRUCT_H_
