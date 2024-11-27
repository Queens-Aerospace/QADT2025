// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/SensorBaro.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__SENSOR_BARO__STRUCT_H_
#define PX4__MSG__DETAIL__SENSOR_BARO__STRUCT_H_

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
  px4__msg__SensorBaro__ORB_QUEUE_LENGTH = 4
};

/// Struct defined in msg/SensorBaro in the package px4.
typedef struct px4__msg__SensorBaro
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  uint64_t timestamp_sample;
  /// unique device ID for the sensor that does not change between power cycles
  uint32_t device_id;
  /// static pressure measurement in Pascals
  float pressure;
  /// temperature in degrees Celsius
  float temperature;
  uint32_t error_count;
} px4__msg__SensorBaro;

// Struct for a sequence of px4__msg__SensorBaro.
typedef struct px4__msg__SensorBaro__Sequence
{
  px4__msg__SensorBaro * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__SensorBaro__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__SENSOR_BARO__STRUCT_H_
