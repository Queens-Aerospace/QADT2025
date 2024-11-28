// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/RoverAckermannSetpoint.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rover_ackermann_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
px4_msgs__msg__RoverAckermannSetpoint__init(px4_msgs__msg__RoverAckermannSetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // forward_speed_setpoint
  // forward_speed_setpoint_normalized
  // steering_setpoint
  // steering_setpoint_normalized
  // lateral_acceleration_setpoint
  return true;
}

void
px4_msgs__msg__RoverAckermannSetpoint__fini(px4_msgs__msg__RoverAckermannSetpoint * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // forward_speed_setpoint
  // forward_speed_setpoint_normalized
  // steering_setpoint
  // steering_setpoint_normalized
  // lateral_acceleration_setpoint
}

bool
px4_msgs__msg__RoverAckermannSetpoint__are_equal(const px4_msgs__msg__RoverAckermannSetpoint * lhs, const px4_msgs__msg__RoverAckermannSetpoint * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // timestamp
  if (lhs->timestamp != rhs->timestamp) {
    return false;
  }
  // forward_speed_setpoint
  if (lhs->forward_speed_setpoint != rhs->forward_speed_setpoint) {
    return false;
  }
  // forward_speed_setpoint_normalized
  if (lhs->forward_speed_setpoint_normalized != rhs->forward_speed_setpoint_normalized) {
    return false;
  }
  // steering_setpoint
  if (lhs->steering_setpoint != rhs->steering_setpoint) {
    return false;
  }
  // steering_setpoint_normalized
  if (lhs->steering_setpoint_normalized != rhs->steering_setpoint_normalized) {
    return false;
  }
  // lateral_acceleration_setpoint
  if (lhs->lateral_acceleration_setpoint != rhs->lateral_acceleration_setpoint) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__RoverAckermannSetpoint__copy(
  const px4_msgs__msg__RoverAckermannSetpoint * input,
  px4_msgs__msg__RoverAckermannSetpoint * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // forward_speed_setpoint
  output->forward_speed_setpoint = input->forward_speed_setpoint;
  // forward_speed_setpoint_normalized
  output->forward_speed_setpoint_normalized = input->forward_speed_setpoint_normalized;
  // steering_setpoint
  output->steering_setpoint = input->steering_setpoint;
  // steering_setpoint_normalized
  output->steering_setpoint_normalized = input->steering_setpoint_normalized;
  // lateral_acceleration_setpoint
  output->lateral_acceleration_setpoint = input->lateral_acceleration_setpoint;
  return true;
}

px4_msgs__msg__RoverAckermannSetpoint *
px4_msgs__msg__RoverAckermannSetpoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannSetpoint * msg = (px4_msgs__msg__RoverAckermannSetpoint *)allocator.allocate(sizeof(px4_msgs__msg__RoverAckermannSetpoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__RoverAckermannSetpoint));
  bool success = px4_msgs__msg__RoverAckermannSetpoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__RoverAckermannSetpoint__destroy(px4_msgs__msg__RoverAckermannSetpoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    px4_msgs__msg__RoverAckermannSetpoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
px4_msgs__msg__RoverAckermannSetpoint__Sequence__init(px4_msgs__msg__RoverAckermannSetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannSetpoint * data = NULL;

  if (size) {
    data = (px4_msgs__msg__RoverAckermannSetpoint *)allocator.zero_allocate(size, sizeof(px4_msgs__msg__RoverAckermannSetpoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__RoverAckermannSetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__RoverAckermannSetpoint__fini(&data[i - 1]);
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
px4_msgs__msg__RoverAckermannSetpoint__Sequence__fini(px4_msgs__msg__RoverAckermannSetpoint__Sequence * array)
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
      px4_msgs__msg__RoverAckermannSetpoint__fini(&array->data[i]);
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

px4_msgs__msg__RoverAckermannSetpoint__Sequence *
px4_msgs__msg__RoverAckermannSetpoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverAckermannSetpoint__Sequence * array = (px4_msgs__msg__RoverAckermannSetpoint__Sequence *)allocator.allocate(sizeof(px4_msgs__msg__RoverAckermannSetpoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__RoverAckermannSetpoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__RoverAckermannSetpoint__Sequence__destroy(px4_msgs__msg__RoverAckermannSetpoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    px4_msgs__msg__RoverAckermannSetpoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
px4_msgs__msg__RoverAckermannSetpoint__Sequence__are_equal(const px4_msgs__msg__RoverAckermannSetpoint__Sequence * lhs, const px4_msgs__msg__RoverAckermannSetpoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__RoverAckermannSetpoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__RoverAckermannSetpoint__Sequence__copy(
  const px4_msgs__msg__RoverAckermannSetpoint__Sequence * input,
  px4_msgs__msg__RoverAckermannSetpoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__RoverAckermannSetpoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    px4_msgs__msg__RoverAckermannSetpoint * data =
      (px4_msgs__msg__RoverAckermannSetpoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__RoverAckermannSetpoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__RoverAckermannSetpoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!px4_msgs__msg__RoverAckermannSetpoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
