// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from px4_msgs:msg/RoverMecanumStatus.idl
// generated code does not contain a copyright notice
#include "px4_msgs/msg/detail/rover_mecanum_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
px4_msgs__msg__RoverMecanumStatus__init(px4_msgs__msg__RoverMecanumStatus * msg)
{
  if (!msg) {
    return false;
  }
  // timestamp
  // measured_forward_speed
  // measured_lateral_speed
  // adjusted_yaw_rate_setpoint
  // measured_yaw_rate
  // measured_yaw
  // pid_yaw_rate_integral
  // pid_yaw_integral
  // pid_forward_throttle_integral
  // pid_lateral_throttle_integral
  return true;
}

void
px4_msgs__msg__RoverMecanumStatus__fini(px4_msgs__msg__RoverMecanumStatus * msg)
{
  if (!msg) {
    return;
  }
  // timestamp
  // measured_forward_speed
  // measured_lateral_speed
  // adjusted_yaw_rate_setpoint
  // measured_yaw_rate
  // measured_yaw
  // pid_yaw_rate_integral
  // pid_yaw_integral
  // pid_forward_throttle_integral
  // pid_lateral_throttle_integral
}

bool
px4_msgs__msg__RoverMecanumStatus__are_equal(const px4_msgs__msg__RoverMecanumStatus * lhs, const px4_msgs__msg__RoverMecanumStatus * rhs)
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
  // measured_lateral_speed
  if (lhs->measured_lateral_speed != rhs->measured_lateral_speed) {
    return false;
  }
  // adjusted_yaw_rate_setpoint
  if (lhs->adjusted_yaw_rate_setpoint != rhs->adjusted_yaw_rate_setpoint) {
    return false;
  }
  // measured_yaw_rate
  if (lhs->measured_yaw_rate != rhs->measured_yaw_rate) {
    return false;
  }
  // measured_yaw
  if (lhs->measured_yaw != rhs->measured_yaw) {
    return false;
  }
  // pid_yaw_rate_integral
  if (lhs->pid_yaw_rate_integral != rhs->pid_yaw_rate_integral) {
    return false;
  }
  // pid_yaw_integral
  if (lhs->pid_yaw_integral != rhs->pid_yaw_integral) {
    return false;
  }
  // pid_forward_throttle_integral
  if (lhs->pid_forward_throttle_integral != rhs->pid_forward_throttle_integral) {
    return false;
  }
  // pid_lateral_throttle_integral
  if (lhs->pid_lateral_throttle_integral != rhs->pid_lateral_throttle_integral) {
    return false;
  }
  return true;
}

bool
px4_msgs__msg__RoverMecanumStatus__copy(
  const px4_msgs__msg__RoverMecanumStatus * input,
  px4_msgs__msg__RoverMecanumStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // timestamp
  output->timestamp = input->timestamp;
  // measured_forward_speed
  output->measured_forward_speed = input->measured_forward_speed;
  // measured_lateral_speed
  output->measured_lateral_speed = input->measured_lateral_speed;
  // adjusted_yaw_rate_setpoint
  output->adjusted_yaw_rate_setpoint = input->adjusted_yaw_rate_setpoint;
  // measured_yaw_rate
  output->measured_yaw_rate = input->measured_yaw_rate;
  // measured_yaw
  output->measured_yaw = input->measured_yaw;
  // pid_yaw_rate_integral
  output->pid_yaw_rate_integral = input->pid_yaw_rate_integral;
  // pid_yaw_integral
  output->pid_yaw_integral = input->pid_yaw_integral;
  // pid_forward_throttle_integral
  output->pid_forward_throttle_integral = input->pid_forward_throttle_integral;
  // pid_lateral_throttle_integral
  output->pid_lateral_throttle_integral = input->pid_lateral_throttle_integral;
  return true;
}

px4_msgs__msg__RoverMecanumStatus *
px4_msgs__msg__RoverMecanumStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverMecanumStatus * msg = (px4_msgs__msg__RoverMecanumStatus *)allocator.allocate(sizeof(px4_msgs__msg__RoverMecanumStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(px4_msgs__msg__RoverMecanumStatus));
  bool success = px4_msgs__msg__RoverMecanumStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
px4_msgs__msg__RoverMecanumStatus__destroy(px4_msgs__msg__RoverMecanumStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    px4_msgs__msg__RoverMecanumStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
px4_msgs__msg__RoverMecanumStatus__Sequence__init(px4_msgs__msg__RoverMecanumStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverMecanumStatus * data = NULL;

  if (size) {
    data = (px4_msgs__msg__RoverMecanumStatus *)allocator.zero_allocate(size, sizeof(px4_msgs__msg__RoverMecanumStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = px4_msgs__msg__RoverMecanumStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        px4_msgs__msg__RoverMecanumStatus__fini(&data[i - 1]);
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
px4_msgs__msg__RoverMecanumStatus__Sequence__fini(px4_msgs__msg__RoverMecanumStatus__Sequence * array)
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
      px4_msgs__msg__RoverMecanumStatus__fini(&array->data[i]);
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

px4_msgs__msg__RoverMecanumStatus__Sequence *
px4_msgs__msg__RoverMecanumStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  px4_msgs__msg__RoverMecanumStatus__Sequence * array = (px4_msgs__msg__RoverMecanumStatus__Sequence *)allocator.allocate(sizeof(px4_msgs__msg__RoverMecanumStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = px4_msgs__msg__RoverMecanumStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
px4_msgs__msg__RoverMecanumStatus__Sequence__destroy(px4_msgs__msg__RoverMecanumStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    px4_msgs__msg__RoverMecanumStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
px4_msgs__msg__RoverMecanumStatus__Sequence__are_equal(const px4_msgs__msg__RoverMecanumStatus__Sequence * lhs, const px4_msgs__msg__RoverMecanumStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!px4_msgs__msg__RoverMecanumStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
px4_msgs__msg__RoverMecanumStatus__Sequence__copy(
  const px4_msgs__msg__RoverMecanumStatus__Sequence * input,
  px4_msgs__msg__RoverMecanumStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(px4_msgs__msg__RoverMecanumStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    px4_msgs__msg__RoverMecanumStatus * data =
      (px4_msgs__msg__RoverMecanumStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!px4_msgs__msg__RoverMecanumStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          px4_msgs__msg__RoverMecanumStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!px4_msgs__msg__RoverMecanumStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
