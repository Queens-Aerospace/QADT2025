// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/WorldControl.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/world_control__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `reset`
#include "ros_gz_interfaces/msg/detail/world_reset__functions.h"
// Member `run_to_sim_time`
#include "builtin_interfaces/msg/detail/time__functions.h"

bool
ros_gz_interfaces__msg__WorldControl__init(ros_gz_interfaces__msg__WorldControl * msg)
{
  if (!msg) {
    return false;
  }
  // pause
  // step
  // multi_step
  msg->multi_step = 0ul;
  // reset
  if (!ros_gz_interfaces__msg__WorldReset__init(&msg->reset)) {
    ros_gz_interfaces__msg__WorldControl__fini(msg);
    return false;
  }
  // seed
  // run_to_sim_time
  if (!builtin_interfaces__msg__Time__init(&msg->run_to_sim_time)) {
    ros_gz_interfaces__msg__WorldControl__fini(msg);
    return false;
  }
  return true;
}

void
ros_gz_interfaces__msg__WorldControl__fini(ros_gz_interfaces__msg__WorldControl * msg)
{
  if (!msg) {
    return;
  }
  // pause
  // step
  // multi_step
  // reset
  ros_gz_interfaces__msg__WorldReset__fini(&msg->reset);
  // seed
  // run_to_sim_time
  builtin_interfaces__msg__Time__fini(&msg->run_to_sim_time);
}

bool
ros_gz_interfaces__msg__WorldControl__are_equal(const ros_gz_interfaces__msg__WorldControl * lhs, const ros_gz_interfaces__msg__WorldControl * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // pause
  if (lhs->pause != rhs->pause) {
    return false;
  }
  // step
  if (lhs->step != rhs->step) {
    return false;
  }
  // multi_step
  if (lhs->multi_step != rhs->multi_step) {
    return false;
  }
  // reset
  if (!ros_gz_interfaces__msg__WorldReset__are_equal(
      &(lhs->reset), &(rhs->reset)))
  {
    return false;
  }
  // seed
  if (lhs->seed != rhs->seed) {
    return false;
  }
  // run_to_sim_time
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->run_to_sim_time), &(rhs->run_to_sim_time)))
  {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__WorldControl__copy(
  const ros_gz_interfaces__msg__WorldControl * input,
  ros_gz_interfaces__msg__WorldControl * output)
{
  if (!input || !output) {
    return false;
  }
  // pause
  output->pause = input->pause;
  // step
  output->step = input->step;
  // multi_step
  output->multi_step = input->multi_step;
  // reset
  if (!ros_gz_interfaces__msg__WorldReset__copy(
      &(input->reset), &(output->reset)))
  {
    return false;
  }
  // seed
  output->seed = input->seed;
  // run_to_sim_time
  if (!builtin_interfaces__msg__Time__copy(
      &(input->run_to_sim_time), &(output->run_to_sim_time)))
  {
    return false;
  }
  return true;
}

ros_gz_interfaces__msg__WorldControl *
ros_gz_interfaces__msg__WorldControl__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__WorldControl * msg = (ros_gz_interfaces__msg__WorldControl *)allocator.allocate(sizeof(ros_gz_interfaces__msg__WorldControl), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__WorldControl));
  bool success = ros_gz_interfaces__msg__WorldControl__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__WorldControl__destroy(ros_gz_interfaces__msg__WorldControl * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__WorldControl__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__WorldControl__Sequence__init(ros_gz_interfaces__msg__WorldControl__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__WorldControl * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__WorldControl *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__WorldControl), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__WorldControl__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__WorldControl__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__WorldControl__Sequence__fini(ros_gz_interfaces__msg__WorldControl__Sequence * array)
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
      ros_gz_interfaces__msg__WorldControl__fini(&array->data[i]);
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

ros_gz_interfaces__msg__WorldControl__Sequence *
ros_gz_interfaces__msg__WorldControl__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__WorldControl__Sequence * array = (ros_gz_interfaces__msg__WorldControl__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__WorldControl__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__WorldControl__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__WorldControl__Sequence__destroy(ros_gz_interfaces__msg__WorldControl__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__WorldControl__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__WorldControl__Sequence__are_equal(const ros_gz_interfaces__msg__WorldControl__Sequence * lhs, const ros_gz_interfaces__msg__WorldControl__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__WorldControl__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__WorldControl__Sequence__copy(
  const ros_gz_interfaces__msg__WorldControl__Sequence * input,
  ros_gz_interfaces__msg__WorldControl__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__WorldControl);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__WorldControl * data =
      (ros_gz_interfaces__msg__WorldControl *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__WorldControl__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__WorldControl__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__WorldControl__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
