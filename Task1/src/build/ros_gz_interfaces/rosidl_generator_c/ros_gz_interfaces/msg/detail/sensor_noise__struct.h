// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ros_gz_interfaces:msg/SensorNoise.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__STRUCT_H_
#define ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'NONE'.
/**
  * Noise type
 */
enum
{
  ros_gz_interfaces__msg__SensorNoise__NONE = 0
};

/// Constant 'GAUSSIAN'.
enum
{
  ros_gz_interfaces__msg__SensorNoise__GAUSSIAN = 2
};

/// Constant 'GAUSSIAN_QUANTIZED'.
enum
{
  ros_gz_interfaces__msg__SensorNoise__GAUSSIAN_QUANTIZED = 3
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"

/// Struct defined in msg/SensorNoise in the package ros_gz_interfaces.
/**
  * A message for specifying sensor noise.
 */
typedef struct ros_gz_interfaces__msg__SensorNoise
{
  /// Optional header data.
  std_msgs__msg__Header header;
  /// The type of noise
  uint8_t type;
  /// Noise mean
  /// Used by GAUSSIAN and GAUSSIAN_QUANTIZED
  double mean;
  /// Noise standard deviation
  /// Used by GAUSSIAN and GAUSSIAN_QUANTIZED
  double stddev;
  /// Noise mean bias
  /// Used by GAUSSIAN and GAUSSIAN_QUANTIZED
  double bias_mean;
  /// Noise standard deviation bias
  double bias_stddev;
  /// Noise precision
  /// Used by GAUSSIAN_QUANTIZED
  double precision;
  /// For type "gaussian*", the standard deviation of the noise used to
  /// drive a process to model slow variations in a sensor bias.
  double dynamic_bias_stddev;
  /// For type "gaussian*", the correlation time in seconds of the
  /// noise used to drive a process to model slow variations in a sensor bias.
  /// A typical value, when used, would be on the order of
  /// 3600 seconds (1 hour).
  double dynamic_bias_correlation_time;
} ros_gz_interfaces__msg__SensorNoise;

// Struct for a sequence of ros_gz_interfaces__msg__SensorNoise.
typedef struct ros_gz_interfaces__msg__SensorNoise__Sequence
{
  ros_gz_interfaces__msg__SensorNoise * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ros_gz_interfaces__msg__SensorNoise__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__STRUCT_H_
