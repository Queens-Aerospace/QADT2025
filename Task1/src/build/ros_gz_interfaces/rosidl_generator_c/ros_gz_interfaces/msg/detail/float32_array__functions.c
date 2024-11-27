// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/Float32Array.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/float32_array__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
ros_gz_interfaces__msg__Float32Array__init(ros_gz_interfaces__msg__Float32Array * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__init(&msg->data, 0)) {
    ros_gz_interfaces__msg__Float32Array__fini(msg);
    return false;
  }
  return true;
}

void
ros_gz_interfaces__msg__Float32Array__fini(ros_gz_interfaces__msg__Float32Array * msg)
{
  if (!msg) {
    return;
  }
  // data
  rosidl_runtime_c__float__Sequence__fini(&msg->data);
}

bool
ros_gz_interfaces__msg__Float32Array__are_equal(const ros_gz_interfaces__msg__Float32Array * lhs, const ros_gz_interfaces__msg__Float32Array * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__Float32Array__copy(
  const ros_gz_interfaces__msg__Float32Array * input,
  ros_gz_interfaces__msg__Float32Array * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

ros_gz_interfaces__msg__Float32Array *
ros_gz_interfaces__msg__Float32Array__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Float32Array * msg = (ros_gz_interfaces__msg__Float32Array *)allocator.allocate(sizeof(ros_gz_interfaces__msg__Float32Array), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__Float32Array));
  bool success = ros_gz_interfaces__msg__Float32Array__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__Float32Array__destroy(ros_gz_interfaces__msg__Float32Array * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__Float32Array__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__Float32Array__Sequence__init(ros_gz_interfaces__msg__Float32Array__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Float32Array * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__Float32Array *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__Float32Array), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__Float32Array__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__Float32Array__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__Float32Array__Sequence__fini(ros_gz_interfaces__msg__Float32Array__Sequence * array)
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
      ros_gz_interfaces__msg__Float32Array__fini(&array->data[i]);
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

ros_gz_interfaces__msg__Float32Array__Sequence *
ros_gz_interfaces__msg__Float32Array__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Float32Array__Sequence * array = (ros_gz_interfaces__msg__Float32Array__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__Float32Array__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__Float32Array__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__Float32Array__Sequence__destroy(ros_gz_interfaces__msg__Float32Array__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__Float32Array__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__Float32Array__Sequence__are_equal(const ros_gz_interfaces__msg__Float32Array__Sequence * lhs, const ros_gz_interfaces__msg__Float32Array__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__Float32Array__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__Float32Array__Sequence__copy(
  const ros_gz_interfaces__msg__Float32Array__Sequence * input,
  ros_gz_interfaces__msg__Float32Array__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__Float32Array);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__Float32Array * data =
      (ros_gz_interfaces__msg__Float32Array *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__Float32Array__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__Float32Array__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__Float32Array__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
