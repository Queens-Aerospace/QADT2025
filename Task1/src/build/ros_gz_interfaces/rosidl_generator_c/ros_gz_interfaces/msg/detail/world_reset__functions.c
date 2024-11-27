// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/WorldReset.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/world_reset__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ros_gz_interfaces__msg__WorldReset__init(ros_gz_interfaces__msg__WorldReset * msg)
{
  if (!msg) {
    return false;
  }
  // all
  msg->all = false;
  // time_only
  msg->time_only = false;
  // model_only
  msg->model_only = false;
  return true;
}

void
ros_gz_interfaces__msg__WorldReset__fini(ros_gz_interfaces__msg__WorldReset * msg)
{
  if (!msg) {
    return;
  }
  // all
  // time_only
  // model_only
}

bool
ros_gz_interfaces__msg__WorldReset__are_equal(const ros_gz_interfaces__msg__WorldReset * lhs, const ros_gz_interfaces__msg__WorldReset * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // all
  if (lhs->all != rhs->all) {
    return false;
  }
  // time_only
  if (lhs->time_only != rhs->time_only) {
    return false;
  }
  // model_only
  if (lhs->model_only != rhs->model_only) {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__WorldReset__copy(
  const ros_gz_interfaces__msg__WorldReset * input,
  ros_gz_interfaces__msg__WorldReset * output)
{
  if (!input || !output) {
    return false;
  }
  // all
  output->all = input->all;
  // time_only
  output->time_only = input->time_only;
  // model_only
  output->model_only = input->model_only;
  return true;
}

ros_gz_interfaces__msg__WorldReset *
ros_gz_interfaces__msg__WorldReset__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__WorldReset * msg = (ros_gz_interfaces__msg__WorldReset *)allocator.allocate(sizeof(ros_gz_interfaces__msg__WorldReset), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__WorldReset));
  bool success = ros_gz_interfaces__msg__WorldReset__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__WorldReset__destroy(ros_gz_interfaces__msg__WorldReset * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__WorldReset__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__WorldReset__Sequence__init(ros_gz_interfaces__msg__WorldReset__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__WorldReset * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__WorldReset *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__WorldReset), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__WorldReset__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__WorldReset__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__WorldReset__Sequence__fini(ros_gz_interfaces__msg__WorldReset__Sequence * array)
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
      ros_gz_interfaces__msg__WorldReset__fini(&array->data[i]);
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

ros_gz_interfaces__msg__WorldReset__Sequence *
ros_gz_interfaces__msg__WorldReset__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__WorldReset__Sequence * array = (ros_gz_interfaces__msg__WorldReset__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__WorldReset__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__WorldReset__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__WorldReset__Sequence__destroy(ros_gz_interfaces__msg__WorldReset__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__WorldReset__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__WorldReset__Sequence__are_equal(const ros_gz_interfaces__msg__WorldReset__Sequence * lhs, const ros_gz_interfaces__msg__WorldReset__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__WorldReset__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__WorldReset__Sequence__copy(
  const ros_gz_interfaces__msg__WorldReset__Sequence * input,
  ros_gz_interfaces__msg__WorldReset__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__WorldReset);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__WorldReset * data =
      (ros_gz_interfaces__msg__WorldReset *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__WorldReset__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__WorldReset__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__WorldReset__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
