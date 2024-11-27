// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/TrackVisual.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/track_visual__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `name`
#include "rosidl_runtime_c/string_functions.h"
// Member `xyz`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
ros_gz_interfaces__msg__TrackVisual__init(ros_gz_interfaces__msg__TrackVisual * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ros_gz_interfaces__msg__TrackVisual__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    ros_gz_interfaces__msg__TrackVisual__fini(msg);
    return false;
  }
  // id
  // inherit_orientation
  // min_dist
  // max_dist
  // is_static
  // use_model_frame
  // xyz
  if (!geometry_msgs__msg__Vector3__init(&msg->xyz)) {
    ros_gz_interfaces__msg__TrackVisual__fini(msg);
    return false;
  }
  // inherit_yaw
  return true;
}

void
ros_gz_interfaces__msg__TrackVisual__fini(ros_gz_interfaces__msg__TrackVisual * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // id
  // inherit_orientation
  // min_dist
  // max_dist
  // is_static
  // use_model_frame
  // xyz
  geometry_msgs__msg__Vector3__fini(&msg->xyz);
  // inherit_yaw
}

bool
ros_gz_interfaces__msg__TrackVisual__are_equal(const ros_gz_interfaces__msg__TrackVisual * lhs, const ros_gz_interfaces__msg__TrackVisual * rhs)
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
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // inherit_orientation
  if (lhs->inherit_orientation != rhs->inherit_orientation) {
    return false;
  }
  // min_dist
  if (lhs->min_dist != rhs->min_dist) {
    return false;
  }
  // max_dist
  if (lhs->max_dist != rhs->max_dist) {
    return false;
  }
  // is_static
  if (lhs->is_static != rhs->is_static) {
    return false;
  }
  // use_model_frame
  if (lhs->use_model_frame != rhs->use_model_frame) {
    return false;
  }
  // xyz
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->xyz), &(rhs->xyz)))
  {
    return false;
  }
  // inherit_yaw
  if (lhs->inherit_yaw != rhs->inherit_yaw) {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__TrackVisual__copy(
  const ros_gz_interfaces__msg__TrackVisual * input,
  ros_gz_interfaces__msg__TrackVisual * output)
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
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // id
  output->id = input->id;
  // inherit_orientation
  output->inherit_orientation = input->inherit_orientation;
  // min_dist
  output->min_dist = input->min_dist;
  // max_dist
  output->max_dist = input->max_dist;
  // is_static
  output->is_static = input->is_static;
  // use_model_frame
  output->use_model_frame = input->use_model_frame;
  // xyz
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->xyz), &(output->xyz)))
  {
    return false;
  }
  // inherit_yaw
  output->inherit_yaw = input->inherit_yaw;
  return true;
}

ros_gz_interfaces__msg__TrackVisual *
ros_gz_interfaces__msg__TrackVisual__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__TrackVisual * msg = (ros_gz_interfaces__msg__TrackVisual *)allocator.allocate(sizeof(ros_gz_interfaces__msg__TrackVisual), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__TrackVisual));
  bool success = ros_gz_interfaces__msg__TrackVisual__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__TrackVisual__destroy(ros_gz_interfaces__msg__TrackVisual * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__TrackVisual__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__TrackVisual__Sequence__init(ros_gz_interfaces__msg__TrackVisual__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__TrackVisual * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__TrackVisual *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__TrackVisual), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__TrackVisual__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__TrackVisual__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__TrackVisual__Sequence__fini(ros_gz_interfaces__msg__TrackVisual__Sequence * array)
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
      ros_gz_interfaces__msg__TrackVisual__fini(&array->data[i]);
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

ros_gz_interfaces__msg__TrackVisual__Sequence *
ros_gz_interfaces__msg__TrackVisual__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__TrackVisual__Sequence * array = (ros_gz_interfaces__msg__TrackVisual__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__TrackVisual__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__TrackVisual__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__TrackVisual__Sequence__destroy(ros_gz_interfaces__msg__TrackVisual__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__TrackVisual__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__TrackVisual__Sequence__are_equal(const ros_gz_interfaces__msg__TrackVisual__Sequence * lhs, const ros_gz_interfaces__msg__TrackVisual__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__TrackVisual__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__TrackVisual__Sequence__copy(
  const ros_gz_interfaces__msg__TrackVisual__Sequence * input,
  ros_gz_interfaces__msg__TrackVisual__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__TrackVisual);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__TrackVisual * data =
      (ros_gz_interfaces__msg__TrackVisual *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__TrackVisual__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__TrackVisual__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__TrackVisual__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
