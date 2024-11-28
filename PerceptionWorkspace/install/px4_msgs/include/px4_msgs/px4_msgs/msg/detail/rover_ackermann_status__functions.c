// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/RoverAckermannStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rover_ackermann_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
px4_msgs__msg__RoverAckermannStatus__init(px4_msgs__msg__RoverAckermannStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // measured_forward_speed
  // adjusted_forward_speed_setpoint
  // steering_setpoint_normalized
  // adjusted_steering_setpoint_normalized
  // measured_lateral_acceleration
  // pid_throttle_integral
  // pid_lat_accel_integral
  return true;
}

void
px4_msgs__msg__RoverAckermannStatus__fini(px4_msgs__msg__RoverAckermannStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // measured_forward_speed
  // adjusted_forward_speed_setpoint
  // steering_setpoint_normalized
  // adjusted_steering_setpoint_normalized
  // measured_lateral_acceleration
  // pid_throttle_integral
  // pid_lat_accel_integral
}

bool
px4_msgs__msg__RoverAckermannStatus__are_equal(const px4_msgs__msg__RoverAckermannStatus * lhs, const px4_msgs__msg__RoverAckermannStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // measured_forward_speed
  if (lhs->measured_forward_speed != rhs->measured_forward_speed) {
    return false;
  }
  // adjusted_forward_speed_setpoint
  if (lhs->adjusted_forward_speed_setpoint != rhs->adjusted_forward_speed_setpoint) {
    return false;
  }
  // steering_setpoint_normalized
  if (lhs->steering_setpoint_normalized != rhs->steering_setpoint_normalized) {
    return false;
  }
  // adjusted_steering_setpoint_normalized
  if (lhs->adjusted_steering_setpoint_normalized != rhs->adjusted_steering_setpoint_normalized) {
    return false;
  }
  // measured_lateral_acceleration
  if (lhs->measured_lateral_acceleration != rhs->measured_lateral_acceleration) {
    return false;
  }
  // pid_throttle_integral
  if (lhs->pid_throttle_integral != rhs->pid_throttle_integral) {
    return false;
  }
  // pid_lat_accel_integral
  if (lhs->pid_lat_accel_integral != rhs->pid_lat_accel_integral) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__RoverAckermannStatus__copy(
  const px4_msgs__msg__RoverAckermannStatus * input,
  px4_msgs__msg__RoverAckermannStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // measured_forward_speed
  output->measured_forward_speed = input->measured_forward_speed;
  // adjusted_forward_speed_setpoint
  output->adjusted_forward_speed_setpoint = input->adjusted_forward_speed_setpoint;
  // steering_setpoint_normalized
  output->steering_setpoint_normalized = input->steering_setpoint_normalized;
  // adjusted_steering_setpoint_normalized
  output->adjusted_steering_setpoint_normalized = input->adjusted_steering_setpoint_normalized;
  // measured_lateral_acceleration
  output->measured_lateral_acceleration = input->measured_lateral_acceleration;
  // pid_throttle_integral
  output->pid_throttle_integral = input->pid_throttle_integral;
  // pid_lat_accel_integral
  output->pid_lat_accel_integral = input->pid_lat_accel_integral;
  return true;
}

px4_msgs__msg__RoverAckermannStatus *
px4_msgs__msg__RoverAckermannStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannStatus * msg = (px4_msgs__msg__RoverAckermannStatus *)allocator.allocate(sizeof(px4_msgs__msg__RoverAckermannStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__RoverAckermannStatus));
  bool success = px4_msgs__msg__RoverAckermannStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__RoverAckermannStatus__destroy(px4_msgs__msg__RoverAckermannStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    px4_msgs__msg__RoverAckermannStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
px4_msgs__msg__RoverAckermannStatus__Sequence__init(px4_msgs__msg__RoverAckermannStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannStatus * data = NULL;

  if (size) {
    data = (px4_msgs__msg__RoverAckermannStatus *)allocator.zero_allocate(size, sizeof(px4_msgs__msg__RoverAckermannStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__RoverAckermannStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__RoverAckermannStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
px4_msgs__msg__RoverAckermannStatus__Sequence__fini(px4_msgs__msg__RoverAckermannStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      px4_msgs__msg__RoverAckermannStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

px4_msgs__msg__RoverAckermannStatus__Sequence *
px4_msgs__msg__RoverAckermannStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannStatus__Sequence * array = (px4_msgs__msg__RoverAckermannStatus__Sequence *)allocator.allocate(sizeof(px4_msgs__msg__RoverAckermannStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__RoverAckermannStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__RoverAckermannStatus__Sequence__destroy(px4_msgs__msg__RoverAckermannStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    px4_msgs__msg__RoverAckermannStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
px4_msgs__msg__RoverAckermannStatus__Sequence__are_equal(const px4_msgs__msg__RoverAckermannStatus__Sequence * lhs, const px4_msgs__msg__RoverAckermannStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__RoverAckermannStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__RoverAckermannStatus__Sequence__copy(
  const px4_msgs__msg__RoverAckermannStatus__Sequence * input,
  px4_msgs__msg__RoverAckermannStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__RoverAckermannStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    px4_msgs__msg__RoverAckermannStatus * data =
      (px4_msgs__msg__RoverAckermannStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__RoverAckermannStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__RoverAckermannStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!px4_msgs__msg__RoverAckermannStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
