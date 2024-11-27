// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/SensorNoise.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/sensor_noise__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ros_gz_interfaces__msg__SensorNoise__init(ros_gz_interfaces__msg__SensorNoise * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ros_gz_interfaces__msg__SensorNoise__fini(msg);
    return false;
  }
  // type
  // mean
  // stddev
  // bias_mean
  // bias_stddev
  // precision
  // dynamic_bias_stddev
  // dynamic_bias_correlation_time
  return true;
}

void
ros_gz_interfaces__msg__SensorNoise__fini(ros_gz_interfaces__msg__SensorNoise * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // type
  // mean
  // stddev
  // bias_mean
  // bias_stddev
  // precision
  // dynamic_bias_stddev
  // dynamic_bias_correlation_time
}

bool
ros_gz_interfaces__msg__SensorNoise__are_equal(const ros_gz_interfaces__msg__SensorNoise * lhs, const ros_gz_interfaces__msg__SensorNoise * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // mean
  if (lhs->mean != rhs->mean) {
    return false;
  }
  // stddev
  if (lhs->stddev != rhs->stddev) {
    return false;
  }
  // bias_mean
  if (lhs->bias_mean != rhs->bias_mean) {
    return false;
  }
  // bias_stddev
  if (lhs->bias_stddev != rhs->bias_stddev) {
    return false;
  }
  // precision
  if (lhs->precision != rhs->precision) {
    return false;
  }
  // dynamic_bias_stddev
  if (lhs->dynamic_bias_stddev != rhs->dynamic_bias_stddev) {
    return false;
  }
  // dynamic_bias_correlation_time
  if (lhs->dynamic_bias_correlation_time != rhs->dynamic_bias_correlation_time) {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__SensorNoise__copy(
  const ros_gz_interfaces__msg__SensorNoise * input,
  ros_gz_interfaces__msg__SensorNoise * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // type
  output->type = input->type;
  // mean
  output->mean = input->mean;
  // stddev
  output->stddev = input->stddev;
  // bias_mean
  output->bias_mean = input->bias_mean;
  // bias_stddev
  output->bias_stddev = input->bias_stddev;
  // precision
  output->precision = input->precision;
  // dynamic_bias_stddev
  output->dynamic_bias_stddev = input->dynamic_bias_stddev;
  // dynamic_bias_correlation_time
  output->dynamic_bias_correlation_time = input->dynamic_bias_correlation_time;
  return true;
}

ros_gz_interfaces__msg__SensorNoise *
ros_gz_interfaces__msg__SensorNoise__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__SensorNoise * msg = (ros_gz_interfaces__msg__SensorNoise *)allocator.allocate(sizeof(ros_gz_interfaces__msg__SensorNoise), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__SensorNoise));
  bool success = ros_gz_interfaces__msg__SensorNoise__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__SensorNoise__destroy(ros_gz_interfaces__msg__SensorNoise * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__SensorNoise__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__SensorNoise__Sequence__init(ros_gz_interfaces__msg__SensorNoise__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__SensorNoise * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__SensorNoise *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__SensorNoise), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__SensorNoise__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__SensorNoise__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__SensorNoise__Sequence__fini(ros_gz_interfaces__msg__SensorNoise__Sequence * array)
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
      ros_gz_interfaces__msg__SensorNoise__fini(&array->data[i]);
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

ros_gz_interfaces__msg__SensorNoise__Sequence *
ros_gz_interfaces__msg__SensorNoise__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__SensorNoise__Sequence * array = (ros_gz_interfaces__msg__SensorNoise__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__SensorNoise__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__SensorNoise__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__SensorNoise__Sequence__destroy(ros_gz_interfaces__msg__SensorNoise__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__SensorNoise__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__SensorNoise__Sequence__are_equal(const ros_gz_interfaces__msg__SensorNoise__Sequence * lhs, const ros_gz_interfaces__msg__SensorNoise__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__SensorNoise__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__SensorNoise__Sequence__copy(
  const ros_gz_interfaces__msg__SensorNoise__Sequence * input,
  ros_gz_interfaces__msg__SensorNoise__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__SensorNoise);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__SensorNoise * data =
      (ros_gz_interfaces__msg__SensorNoise *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__SensorNoise__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__SensorNoise__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__SensorNoise__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
