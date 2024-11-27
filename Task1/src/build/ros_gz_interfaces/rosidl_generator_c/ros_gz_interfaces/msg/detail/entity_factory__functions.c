// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/EntityFactory.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/entity_factory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
// Member `sdf`
// Member `sdf_filename`
// Member `clone_name`
// Member `relative_to`
#include "rosidl_runtime_c/string_functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
ros_gz_interfaces__msg__EntityFactory__init(ros_gz_interfaces__msg__EntityFactory * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    ros_gz_interfaces__msg__EntityFactory__fini(msg);
    return false;
  }
  // allow_renaming
  msg->allow_renaming = false;
  // sdf
  if (!rosidl_runtime_c__String__init(&msg->sdf)) {
    ros_gz_interfaces__msg__EntityFactory__fini(msg);
    return false;
  }
  // sdf_filename
  if (!rosidl_runtime_c__String__init(&msg->sdf_filename)) {
    ros_gz_interfaces__msg__EntityFactory__fini(msg);
    return false;
  }
  // clone_name
  if (!rosidl_runtime_c__String__init(&msg->clone_name)) {
    ros_gz_interfaces__msg__EntityFactory__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    ros_gz_interfaces__msg__EntityFactory__fini(msg);
    return false;
  }
  // relative_to
  if (!rosidl_runtime_c__String__init(&msg->relative_to)) {
    ros_gz_interfaces__msg__EntityFactory__fini(msg);
    return false;
  }
  {
    bool success = rosidl_runtime_c__String__assign(&msg->relative_to, "world");
    if (!success) {
      goto abort_init_0;
    }
  }
  return true;
abort_init_0:
  return false;
}

void
ros_gz_interfaces__msg__EntityFactory__fini(ros_gz_interfaces__msg__EntityFactory * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // allow_renaming
  // sdf
  rosidl_runtime_c__String__fini(&msg->sdf);
  // sdf_filename
  rosidl_runtime_c__String__fini(&msg->sdf_filename);
  // clone_name
  rosidl_runtime_c__String__fini(&msg->clone_name);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // relative_to
  rosidl_runtime_c__String__fini(&msg->relative_to);
}

bool
ros_gz_interfaces__msg__EntityFactory__are_equal(const ros_gz_interfaces__msg__EntityFactory * lhs, const ros_gz_interfaces__msg__EntityFactory * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // allow_renaming
  if (lhs->allow_renaming != rhs->allow_renaming) {
    return false;
  }
  // sdf
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sdf), &(rhs->sdf)))
  {
    return false;
  }
  // sdf_filename
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->sdf_filename), &(rhs->sdf_filename)))
  {
    return false;
  }
  // clone_name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->clone_name), &(rhs->clone_name)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  // relative_to
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->relative_to), &(rhs->relative_to)))
  {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__EntityFactory__copy(
  const ros_gz_interfaces__msg__EntityFactory * input,
  ros_gz_interfaces__msg__EntityFactory * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // allow_renaming
  output->allow_renaming = input->allow_renaming;
  // sdf
  if (!rosidl_runtime_c__String__copy(
      &(input->sdf), &(output->sdf)))
  {
    return false;
  }
  // sdf_filename
  if (!rosidl_runtime_c__String__copy(
      &(input->sdf_filename), &(output->sdf_filename)))
  {
    return false;
  }
  // clone_name
  if (!rosidl_runtime_c__String__copy(
      &(input->clone_name), &(output->clone_name)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  // relative_to
  if (!rosidl_runtime_c__String__copy(
      &(input->relative_to), &(output->relative_to)))
  {
    return false;
  }
  return true;
}

ros_gz_interfaces__msg__EntityFactory *
ros_gz_interfaces__msg__EntityFactory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__EntityFactory * msg = (ros_gz_interfaces__msg__EntityFactory *)allocator.allocate(sizeof(ros_gz_interfaces__msg__EntityFactory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__EntityFactory));
  bool success = ros_gz_interfaces__msg__EntityFactory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__EntityFactory__destroy(ros_gz_interfaces__msg__EntityFactory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__EntityFactory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__EntityFactory__Sequence__init(ros_gz_interfaces__msg__EntityFactory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__EntityFactory * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__EntityFactory *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__EntityFactory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__EntityFactory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__EntityFactory__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__EntityFactory__Sequence__fini(ros_gz_interfaces__msg__EntityFactory__Sequence * array)
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
      ros_gz_interfaces__msg__EntityFactory__fini(&array->data[i]);
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

ros_gz_interfaces__msg__EntityFactory__Sequence *
ros_gz_interfaces__msg__EntityFactory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__EntityFactory__Sequence * array = (ros_gz_interfaces__msg__EntityFactory__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__EntityFactory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__EntityFactory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__EntityFactory__Sequence__destroy(ros_gz_interfaces__msg__EntityFactory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__EntityFactory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__EntityFactory__Sequence__are_equal(const ros_gz_interfaces__msg__EntityFactory__Sequence * lhs, const ros_gz_interfaces__msg__EntityFactory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__EntityFactory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__EntityFactory__Sequence__copy(
  const ros_gz_interfaces__msg__EntityFactory__Sequence * input,
  ros_gz_interfaces__msg__EntityFactory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__EntityFactory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__EntityFactory * data =
      (ros_gz_interfaces__msg__EntityFactory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__EntityFactory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__EntityFactory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__EntityFactory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
