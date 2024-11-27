// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/MaterialColor.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/material_color__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `entity`
#include "ros_gz_interfaces/msg/detail/entity__functions.h"
// Member `ambient`
// Member `diffuse`
// Member `specular`
// Member `emissive`
#include "std_msgs/msg/detail/color_rgba__functions.h"

bool
ros_gz_interfaces__msg__MaterialColor__init(ros_gz_interfaces__msg__MaterialColor * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ros_gz_interfaces__msg__MaterialColor__fini(msg);
    return false;
  }
  // entity
  if (!ros_gz_interfaces__msg__Entity__init(&msg->entity)) {
    ros_gz_interfaces__msg__MaterialColor__fini(msg);
    return false;
  }
  // ambient
  if (!std_msgs__msg__ColorRGBA__init(&msg->ambient)) {
    ros_gz_interfaces__msg__MaterialColor__fini(msg);
    return false;
  }
  // diffuse
  if (!std_msgs__msg__ColorRGBA__init(&msg->diffuse)) {
    ros_gz_interfaces__msg__MaterialColor__fini(msg);
    return false;
  }
  // specular
  if (!std_msgs__msg__ColorRGBA__init(&msg->specular)) {
    ros_gz_interfaces__msg__MaterialColor__fini(msg);
    return false;
  }
  // emissive
  if (!std_msgs__msg__ColorRGBA__init(&msg->emissive)) {
    ros_gz_interfaces__msg__MaterialColor__fini(msg);
    return false;
  }
  // shininess
  // entity_match
  return true;
}

void
ros_gz_interfaces__msg__MaterialColor__fini(ros_gz_interfaces__msg__MaterialColor * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // entity
  ros_gz_interfaces__msg__Entity__fini(&msg->entity);
  // ambient
  std_msgs__msg__ColorRGBA__fini(&msg->ambient);
  // diffuse
  std_msgs__msg__ColorRGBA__fini(&msg->diffuse);
  // specular
  std_msgs__msg__ColorRGBA__fini(&msg->specular);
  // emissive
  std_msgs__msg__ColorRGBA__fini(&msg->emissive);
  // shininess
  // entity_match
}

bool
ros_gz_interfaces__msg__MaterialColor__are_equal(const ros_gz_interfaces__msg__MaterialColor * lhs, const ros_gz_interfaces__msg__MaterialColor * rhs)
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
  // entity
  if (!ros_gz_interfaces__msg__Entity__are_equal(
      &(lhs->entity), &(rhs->entity)))
  {
    return false;
  }
  // ambient
  if (!std_msgs__msg__ColorRGBA__are_equal(
      &(lhs->ambient), &(rhs->ambient)))
  {
    return false;
  }
  // diffuse
  if (!std_msgs__msg__ColorRGBA__are_equal(
      &(lhs->diffuse), &(rhs->diffuse)))
  {
    return false;
  }
  // specular
  if (!std_msgs__msg__ColorRGBA__are_equal(
      &(lhs->specular), &(rhs->specular)))
  {
    return false;
  }
  // emissive
  if (!std_msgs__msg__ColorRGBA__are_equal(
      &(lhs->emissive), &(rhs->emissive)))
  {
    return false;
  }
  // shininess
  if (lhs->shininess != rhs->shininess) {
    return false;
  }
  // entity_match
  if (lhs->entity_match != rhs->entity_match) {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__MaterialColor__copy(
  const ros_gz_interfaces__msg__MaterialColor * input,
  ros_gz_interfaces__msg__MaterialColor * output)
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
  // entity
  if (!ros_gz_interfaces__msg__Entity__copy(
      &(input->entity), &(output->entity)))
  {
    return false;
  }
  // ambient
  if (!std_msgs__msg__ColorRGBA__copy(
      &(input->ambient), &(output->ambient)))
  {
    return false;
  }
  // diffuse
  if (!std_msgs__msg__ColorRGBA__copy(
      &(input->diffuse), &(output->diffuse)))
  {
    return false;
  }
  // specular
  if (!std_msgs__msg__ColorRGBA__copy(
      &(input->specular), &(output->specular)))
  {
    return false;
  }
  // emissive
  if (!std_msgs__msg__ColorRGBA__copy(
      &(input->emissive), &(output->emissive)))
  {
    return false;
  }
  // shininess
  output->shininess = input->shininess;
  // entity_match
  output->entity_match = input->entity_match;
  return true;
}

ros_gz_interfaces__msg__MaterialColor *
ros_gz_interfaces__msg__MaterialColor__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__MaterialColor * msg = (ros_gz_interfaces__msg__MaterialColor *)allocator.allocate(sizeof(ros_gz_interfaces__msg__MaterialColor), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__MaterialColor));
  bool success = ros_gz_interfaces__msg__MaterialColor__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__MaterialColor__destroy(ros_gz_interfaces__msg__MaterialColor * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__MaterialColor__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__MaterialColor__Sequence__init(ros_gz_interfaces__msg__MaterialColor__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__MaterialColor * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__MaterialColor *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__MaterialColor), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__MaterialColor__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__MaterialColor__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__MaterialColor__Sequence__fini(ros_gz_interfaces__msg__MaterialColor__Sequence * array)
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
      ros_gz_interfaces__msg__MaterialColor__fini(&array->data[i]);
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

ros_gz_interfaces__msg__MaterialColor__Sequence *
ros_gz_interfaces__msg__MaterialColor__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__MaterialColor__Sequence * array = (ros_gz_interfaces__msg__MaterialColor__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__MaterialColor__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__MaterialColor__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__MaterialColor__Sequence__destroy(ros_gz_interfaces__msg__MaterialColor__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__MaterialColor__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__MaterialColor__Sequence__are_equal(const ros_gz_interfaces__msg__MaterialColor__Sequence * lhs, const ros_gz_interfaces__msg__MaterialColor__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__MaterialColor__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__MaterialColor__Sequence__copy(
  const ros_gz_interfaces__msg__MaterialColor__Sequence * input,
  ros_gz_interfaces__msg__MaterialColor__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__MaterialColor);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__MaterialColor * data =
      (ros_gz_interfaces__msg__MaterialColor *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__MaterialColor__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__MaterialColor__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__MaterialColor__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
