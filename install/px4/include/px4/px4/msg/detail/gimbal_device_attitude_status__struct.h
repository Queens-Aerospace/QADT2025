// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4:msg/GimbalDeviceAttitudeStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__STRUCT_H_
#define PX4__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'DEVICE_FLAGS_RETRACT'.
enum
{
  px4__msg__GimbalDeviceAttitudeStatus__DEVICE_FLAGS_RETRACT = 1
};

/// Constant 'DEVICE_FLAGS_NEUTRAL'.
enum
{
  px4__msg__GimbalDeviceAttitudeStatus__DEVICE_FLAGS_NEUTRAL = 2
};

/// Constant 'DEVICE_FLAGS_ROLL_LOCK'.
enum
{
  px4__msg__GimbalDeviceAttitudeStatus__DEVICE_FLAGS_ROLL_LOCK = 4
};

/// Constant 'DEVICE_FLAGS_PITCH_LOCK'.
enum
{
  px4__msg__GimbalDeviceAttitudeStatus__DEVICE_FLAGS_PITCH_LOCK = 8
};

/// Constant 'DEVICE_FLAGS_YAW_LOCK'.
enum
{
  px4__msg__GimbalDeviceAttitudeStatus__DEVICE_FLAGS_YAW_LOCK = 16
};

/// Struct defined in msg/GimbalDeviceAttitudeStatus in the package px4.
typedef struct px4__msg__GimbalDeviceAttitudeStatus
{
  /// time since system start (microseconds)
  uint64_t timestamp;
  uint8_t target_system;
  uint8_t target_component;
  uint16_t device_flags;
  float q[4];
  float angular_velocity_x;
  float angular_velocity_y;
  float angular_velocity_z;
  uint32_t failure_flags;
  bool received_from_mavlink;
} px4__msg__GimbalDeviceAttitudeStatus;

// Struct for a sequence of px4__msg__GimbalDeviceAttitudeStatus.
typedef struct px4__msg__GimbalDeviceAttitudeStatus__Sequence
{
  px4__msg__GimbalDeviceAttitudeStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4__msg__GimbalDeviceAttitudeStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4__MSG__DETAIL__GIMBAL_DEVICE_ATTITUDE_STATUS__STRUCT_H_
