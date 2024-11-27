// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/Contact.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/contact__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `collision1`
// Member `collision2`
#include "ros_gz_interfaces/msg/detail/entity__functions.h"
// Member `positions`
// Member `normals`
#include "geometry_msgs/msg/detail/vector3__functions.h"
// Member `depths`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `wrenches`
#include "ros_gz_interfaces/msg/detail/joint_wrench__functions.h"

bool
ros_gz_interfaces__msg__Contact__init(ros_gz_interfaces__msg__Contact * msg)
{
  if (!msg) {
    return false;
  }
  // collision1
  if (!ros_gz_interfaces__msg__Entity__init(&msg->collision1)) {
    ros_gz_interfaces__msg__Contact__fini(msg);
    return false;
  }
  // collision2
  if (!ros_gz_interfaces__msg__Entity__init(&msg->collision2)) {
    ros_gz_interfaces__msg__Contact__fini(msg);
    return false;
  }
  // positions
  if (!geometry_msgs__msg__Vector3__Sequence__init(&msg->positions, 0)) {
    ros_gz_interfaces__msg__Contact__fini(msg);
    return false;
  }
  // normals
  if (!geometry_msgs__msg__Vector3__Sequence__init(&msg->normals, 0)) {
    ros_gz_interfaces__msg__Contact__fini(msg);
    return false;
  }
  // depths
  if (!rosidl_runtime_c__double__Sequence__init(&msg->depths, 0)) {
    ros_gz_interfaces__msg__Contact__fini(msg);
    return false;
  }
  // wrenches
  if (!ros_gz_interfaces__msg__JointWrench__Sequence__init(&msg->wrenches, 0)) {
    ros_gz_interfaces__msg__Contact__fini(msg);
    return false;
  }
  return true;
}

void
ros_gz_interfaces__msg__Contact__fini(ros_gz_interfaces__msg__Contact * msg)
{
  if (!msg) {
    return;
  }
  // collision1
  ros_gz_interfaces__msg__Entity__fini(&msg->collision1);
  // collision2
  ros_gz_interfaces__msg__Entity__fini(&msg->collision2);
  // positions
  geometry_msgs__msg__Vector3__Sequence__fini(&msg->positions);
  // normals
  geometry_msgs__msg__Vector3__Sequence__fini(&msg->normals);
  // depths
  rosidl_runtime_c__double__Sequence__fini(&msg->depths);
  // wrenches
  ros_gz_interfaces__msg__JointWrench__Sequence__fini(&msg->wrenches);
}

bool
ros_gz_interfaces__msg__Contact__are_equal(const ros_gz_interfaces__msg__Contact * lhs, const ros_gz_interfaces__msg__Contact * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // collision1
  if (!ros_gz_interfaces__msg__Entity__are_equal(
      &(lhs->collision1), &(rhs->collision1)))
  {
    return false;
  }
  // collision2
  if (!ros_gz_interfaces__msg__Entity__are_equal(
      &(lhs->collision2), &(rhs->collision2)))
  {
    return false;
  }
  // positions
  if (!geometry_msgs__msg__Vector3__Sequence__are_equal(
      &(lhs->positions), &(rhs->positions)))
  {
    return false;
  }
  // normals
  if (!geometry_msgs__msg__Vector3__Sequence__are_equal(
      &(lhs->normals), &(rhs->normals)))
  {
    return false;
  }
  // depths
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->depths), &(rhs->depths)))
  {
    return false;
  }
  // wrenches
  if (!ros_gz_interfaces__msg__JointWrench__Sequence__are_equal(
      &(lhs->wrenches), &(rhs->wrenches)))
  {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__Contact__copy(
  const ros_gz_interfaces__msg__Contact * input,
  ros_gz_interfaces__msg__Contact * output)
{
  if (!input || !output) {
    return false;
  }
  // collision1
  if (!ros_gz_interfaces__msg__Entity__copy(
      &(input->collision1), &(output->collision1)))
  {
    return false;
  }
  // collision2
  if (!ros_gz_interfaces__msg__Entity__copy(
      &(input->collision2), &(output->collision2)))
  {
    return false;
  }
  // positions
  if (!geometry_msgs__msg__Vector3__Sequence__copy(
      &(input->positions), &(output->positions)))
  {
    return false;
  }
  // normals
  if (!geometry_msgs__msg__Vector3__Sequence__copy(
      &(input->normals), &(output->normals)))
  {
    return false;
  }
  // depths
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->depths), &(output->depths)))
  {
    return false;
  }
  // wrenches
  if (!ros_gz_interfaces__msg__JointWrench__Sequence__copy(
      &(input->wrenches), &(output->wrenches)))
  {
    return false;
  }
  return true;
}

ros_gz_interfaces__msg__Contact *
ros_gz_interfaces__msg__Contact__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Contact * msg = (ros_gz_interfaces__msg__Contact *)allocator.allocate(sizeof(ros_gz_interfaces__msg__Contact), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__Contact));
  bool success = ros_gz_interfaces__msg__Contact__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__Contact__destroy(ros_gz_interfaces__msg__Contact * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__Contact__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__Contact__Sequence__init(ros_gz_interfaces__msg__Contact__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Contact * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__Contact *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__Contact), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__Contact__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__Contact__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__Contact__Sequence__fini(ros_gz_interfaces__msg__Contact__Sequence * array)
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
      ros_gz_interfaces__msg__Contact__fini(&array->data[i]);
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

ros_gz_interfaces__msg__Contact__Sequence *
ros_gz_interfaces__msg__Contact__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Contact__Sequence * array = (ros_gz_interfaces__msg__Contact__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__Contact__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__Contact__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__Contact__Sequence__destroy(ros_gz_interfaces__msg__Contact__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__Contact__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__Contact__Sequence__are_equal(const ros_gz_interfaces__msg__Contact__Sequence * lhs, const ros_gz_interfaces__msg__Contact__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__Contact__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__Contact__Sequence__copy(
  const ros_gz_interfaces__msg__Contact__Sequence * input,
  ros_gz_interfaces__msg__Contact__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__Contact);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__Contact * data =
      (ros_gz_interfaces__msg__Contact *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__Contact__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__Contact__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__Contact__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
