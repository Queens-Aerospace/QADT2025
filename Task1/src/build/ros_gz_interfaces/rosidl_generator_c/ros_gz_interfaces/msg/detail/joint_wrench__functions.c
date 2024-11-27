// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/JointWrench.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/joint_wrench__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `body_1_name`
// Member `body_2_name`
#include "std_msgs/msg/detail/string__functions.h"
// Member `body_1_id`
// Member `body_2_id`
#include "std_msgs/msg/detail/u_int32__functions.h"
// Member `body_1_wrench`
// Member `body_2_wrench`
#include "geometry_msgs/msg/detail/wrench__functions.h"

bool
ros_gz_interfaces__msg__JointWrench__init(ros_gz_interfaces__msg__JointWrench * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
    return false;
  }
  // body_1_name
  if (!std_msgs__msg__String__init(&msg->body_1_name)) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
    return false;
  }
  // body_1_id
  if (!std_msgs__msg__UInt32__init(&msg->body_1_id)) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
    return false;
  }
  // body_2_name
  if (!std_msgs__msg__String__init(&msg->body_2_name)) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
    return false;
  }
  // body_2_id
  if (!std_msgs__msg__UInt32__init(&msg->body_2_id)) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
    return false;
  }
  // body_1_wrench
  if (!geometry_msgs__msg__Wrench__init(&msg->body_1_wrench)) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
    return false;
  }
  // body_2_wrench
  if (!geometry_msgs__msg__Wrench__init(&msg->body_2_wrench)) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
    return false;
  }
  return true;
}

void
ros_gz_interfaces__msg__JointWrench__fini(ros_gz_interfaces__msg__JointWrench * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // body_1_name
  std_msgs__msg__String__fini(&msg->body_1_name);
  // body_1_id
  std_msgs__msg__UInt32__fini(&msg->body_1_id);
  // body_2_name
  std_msgs__msg__String__fini(&msg->body_2_name);
  // body_2_id
  std_msgs__msg__UInt32__fini(&msg->body_2_id);
  // body_1_wrench
  geometry_msgs__msg__Wrench__fini(&msg->body_1_wrench);
  // body_2_wrench
  geometry_msgs__msg__Wrench__fini(&msg->body_2_wrench);
}

bool
ros_gz_interfaces__msg__JointWrench__are_equal(const ros_gz_interfaces__msg__JointWrench * lhs, const ros_gz_interfaces__msg__JointWrench * rhs)
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
  // body_1_name
  if (!std_msgs__msg__String__are_equal(
      &(lhs->body_1_name), &(rhs->body_1_name)))
  {
    return false;
  }
  // body_1_id
  if (!std_msgs__msg__UInt32__are_equal(
      &(lhs->body_1_id), &(rhs->body_1_id)))
  {
    return false;
  }
  // body_2_name
  if (!std_msgs__msg__String__are_equal(
      &(lhs->body_2_name), &(rhs->body_2_name)))
  {
    return false;
  }
  // body_2_id
  if (!std_msgs__msg__UInt32__are_equal(
      &(lhs->body_2_id), &(rhs->body_2_id)))
  {
    return false;
  }
  // body_1_wrench
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->body_1_wrench), &(rhs->body_1_wrench)))
  {
    return false;
  }
  // body_2_wrench
  if (!geometry_msgs__msg__Wrench__are_equal(
      &(lhs->body_2_wrench), &(rhs->body_2_wrench)))
  {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__JointWrench__copy(
  const ros_gz_interfaces__msg__JointWrench * input,
  ros_gz_interfaces__msg__JointWrench * output)
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
  // body_1_name
  if (!std_msgs__msg__String__copy(
      &(input->body_1_name), &(output->body_1_name)))
  {
    return false;
  }
  // body_1_id
  if (!std_msgs__msg__UInt32__copy(
      &(input->body_1_id), &(output->body_1_id)))
  {
    return false;
  }
  // body_2_name
  if (!std_msgs__msg__String__copy(
      &(input->body_2_name), &(output->body_2_name)))
  {
    return false;
  }
  // body_2_id
  if (!std_msgs__msg__UInt32__copy(
      &(input->body_2_id), &(output->body_2_id)))
  {
    return false;
  }
  // body_1_wrench
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->body_1_wrench), &(output->body_1_wrench)))
  {
    return false;
  }
  // body_2_wrench
  if (!geometry_msgs__msg__Wrench__copy(
      &(input->body_2_wrench), &(output->body_2_wrench)))
  {
    return false;
  }
  return true;
}

ros_gz_interfaces__msg__JointWrench *
ros_gz_interfaces__msg__JointWrench__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__JointWrench * msg = (ros_gz_interfaces__msg__JointWrench *)allocator.allocate(sizeof(ros_gz_interfaces__msg__JointWrench), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__JointWrench));
  bool success = ros_gz_interfaces__msg__JointWrench__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__JointWrench__destroy(ros_gz_interfaces__msg__JointWrench * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__JointWrench__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__JointWrench__Sequence__init(ros_gz_interfaces__msg__JointWrench__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__JointWrench * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__JointWrench *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__JointWrench), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__JointWrench__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__JointWrench__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__JointWrench__Sequence__fini(ros_gz_interfaces__msg__JointWrench__Sequence * array)
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
      ros_gz_interfaces__msg__JointWrench__fini(&array->data[i]);
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

ros_gz_interfaces__msg__JointWrench__Sequence *
ros_gz_interfaces__msg__JointWrench__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__JointWrench__Sequence * array = (ros_gz_interfaces__msg__JointWrench__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__JointWrench__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__JointWrench__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__JointWrench__Sequence__destroy(ros_gz_interfaces__msg__JointWrench__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__JointWrench__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__JointWrench__Sequence__are_equal(const ros_gz_interfaces__msg__JointWrench__Sequence * lhs, const ros_gz_interfaces__msg__JointWrench__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__JointWrench__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__JointWrench__Sequence__copy(
  const ros_gz_interfaces__msg__JointWrench__Sequence * input,
  ros_gz_interfaces__msg__JointWrench__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__JointWrench);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__JointWrench * data =
      (ros_gz_interfaces__msg__JointWrench *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__JointWrench__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__JointWrench__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__JointWrench__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
