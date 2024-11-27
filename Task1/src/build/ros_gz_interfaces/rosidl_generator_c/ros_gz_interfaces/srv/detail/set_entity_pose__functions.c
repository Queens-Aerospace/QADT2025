// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:srv/SetEntityPose.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/srv/detail/set_entity_pose__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `entity`
#include "ros_gz_interfaces/msg/detail/entity__functions.h"
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"

bool
ros_gz_interfaces__srv__SetEntityPose_Request__init(ros_gz_interfaces__srv__SetEntityPose_Request * msg)
{
  if (!msg) {
    return false;
  }
  // entity
  if (!ros_gz_interfaces__msg__Entity__init(&msg->entity)) {
    ros_gz_interfaces__srv__SetEntityPose_Request__fini(msg);
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    ros_gz_interfaces__srv__SetEntityPose_Request__fini(msg);
    return false;
  }
  return true;
}

void
ros_gz_interfaces__srv__SetEntityPose_Request__fini(ros_gz_interfaces__srv__SetEntityPose_Request * msg)
{
  if (!msg) {
    return;
  }
  // entity
  ros_gz_interfaces__msg__Entity__fini(&msg->entity);
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
}

bool
ros_gz_interfaces__srv__SetEntityPose_Request__are_equal(const ros_gz_interfaces__srv__SetEntityPose_Request * lhs, const ros_gz_interfaces__srv__SetEntityPose_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // entity
  if (!ros_gz_interfaces__msg__Entity__are_equal(
      &(lhs->entity), &(rhs->entity)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__srv__SetEntityPose_Request__copy(
  const ros_gz_interfaces__srv__SetEntityPose_Request * input,
  ros_gz_interfaces__srv__SetEntityPose_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // entity
  if (!ros_gz_interfaces__msg__Entity__copy(
      &(input->entity), &(output->entity)))
  {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

ros_gz_interfaces__srv__SetEntityPose_Request *
ros_gz_interfaces__srv__SetEntityPose_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__srv__SetEntityPose_Request * msg = (ros_gz_interfaces__srv__SetEntityPose_Request *)allocator.allocate(sizeof(ros_gz_interfaces__srv__SetEntityPose_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__srv__SetEntityPose_Request));
  bool success = ros_gz_interfaces__srv__SetEntityPose_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__srv__SetEntityPose_Request__destroy(ros_gz_interfaces__srv__SetEntityPose_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__srv__SetEntityPose_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__init(ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__srv__SetEntityPose_Request * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__srv__SetEntityPose_Request *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__srv__SetEntityPose_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__srv__SetEntityPose_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__srv__SetEntityPose_Request__fini(&data[i - 1]);
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
ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__fini(ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * array)
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
      ros_gz_interfaces__srv__SetEntityPose_Request__fini(&array->data[i]);
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

ros_gz_interfaces__srv__SetEntityPose_Request__Sequence *
ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * array = (ros_gz_interfaces__srv__SetEntityPose_Request__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__srv__SetEntityPose_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__destroy(ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__are_equal(const ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * lhs, const ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__srv__SetEntityPose_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__srv__SetEntityPose_Request__Sequence__copy(
  const ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * input,
  ros_gz_interfaces__srv__SetEntityPose_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__srv__SetEntityPose_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__srv__SetEntityPose_Request * data =
      (ros_gz_interfaces__srv__SetEntityPose_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__srv__SetEntityPose_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__srv__SetEntityPose_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__srv__SetEntityPose_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
ros_gz_interfaces__srv__SetEntityPose_Response__init(ros_gz_interfaces__srv__SetEntityPose_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  return true;
}

void
ros_gz_interfaces__srv__SetEntityPose_Response__fini(ros_gz_interfaces__srv__SetEntityPose_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
}

bool
ros_gz_interfaces__srv__SetEntityPose_Response__are_equal(const ros_gz_interfaces__srv__SetEntityPose_Response * lhs, const ros_gz_interfaces__srv__SetEntityPose_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__srv__SetEntityPose_Response__copy(
  const ros_gz_interfaces__srv__SetEntityPose_Response * input,
  ros_gz_interfaces__srv__SetEntityPose_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  return true;
}

ros_gz_interfaces__srv__SetEntityPose_Response *
ros_gz_interfaces__srv__SetEntityPose_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__srv__SetEntityPose_Response * msg = (ros_gz_interfaces__srv__SetEntityPose_Response *)allocator.allocate(sizeof(ros_gz_interfaces__srv__SetEntityPose_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__srv__SetEntityPose_Response));
  bool success = ros_gz_interfaces__srv__SetEntityPose_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__srv__SetEntityPose_Response__destroy(ros_gz_interfaces__srv__SetEntityPose_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__srv__SetEntityPose_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__init(ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__srv__SetEntityPose_Response * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__srv__SetEntityPose_Response *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__srv__SetEntityPose_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__srv__SetEntityPose_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__srv__SetEntityPose_Response__fini(&data[i - 1]);
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
ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__fini(ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * array)
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
      ros_gz_interfaces__srv__SetEntityPose_Response__fini(&array->data[i]);
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

ros_gz_interfaces__srv__SetEntityPose_Response__Sequence *
ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * array = (ros_gz_interfaces__srv__SetEntityPose_Response__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__srv__SetEntityPose_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__destroy(ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__are_equal(const ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * lhs, const ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__srv__SetEntityPose_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__srv__SetEntityPose_Response__Sequence__copy(
  const ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * input,
  ros_gz_interfaces__srv__SetEntityPose_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__srv__SetEntityPose_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__srv__SetEntityPose_Response * data =
      (ros_gz_interfaces__srv__SetEntityPose_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__srv__SetEntityPose_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__srv__SetEntityPose_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__srv__SetEntityPose_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
