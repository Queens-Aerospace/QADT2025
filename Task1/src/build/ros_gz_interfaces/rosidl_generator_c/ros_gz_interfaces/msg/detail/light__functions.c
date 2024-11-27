// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ros_gz_interfaces:msg/Light.idl
// generated code does not contain a copyright notice
#include "ros_gz_interfaces/msg/detail/light__functions.h"

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
// Member `pose`
#include "geometry_msgs/msg/detail/pose__functions.h"
// Member `diffuse`
// Member `specular`
#include "std_msgs/msg/detail/color_rgba__functions.h"
// Member `direction`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
ros_gz_interfaces__msg__Light__init(ros_gz_interfaces__msg__Light * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ros_gz_interfaces__msg__Light__fini(msg);
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    ros_gz_interfaces__msg__Light__fini(msg);
    return false;
  }
  // type
  // pose
  if (!geometry_msgs__msg__Pose__init(&msg->pose)) {
    ros_gz_interfaces__msg__Light__fini(msg);
    return false;
  }
  // diffuse
  if (!std_msgs__msg__ColorRGBA__init(&msg->diffuse)) {
    ros_gz_interfaces__msg__Light__fini(msg);
    return false;
  }
  // specular
  if (!std_msgs__msg__ColorRGBA__init(&msg->specular)) {
    ros_gz_interfaces__msg__Light__fini(msg);
    return false;
  }
  // attenuation_constant
  // attenuation_linear
  // attenuation_quadratic
  // direction
  if (!geometry_msgs__msg__Vector3__init(&msg->direction)) {
    ros_gz_interfaces__msg__Light__fini(msg);
    return false;
  }
  // range
  // cast_shadows
  // spot_inner_angle
  // spot_outer_angle
  // spot_falloff
  // id
  // parent_id
  // intensity
  return true;
}

void
ros_gz_interfaces__msg__Light__fini(ros_gz_interfaces__msg__Light * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // type
  // pose
  geometry_msgs__msg__Pose__fini(&msg->pose);
  // diffuse
  std_msgs__msg__ColorRGBA__fini(&msg->diffuse);
  // specular
  std_msgs__msg__ColorRGBA__fini(&msg->specular);
  // attenuation_constant
  // attenuation_linear
  // attenuation_quadratic
  // direction
  geometry_msgs__msg__Vector3__fini(&msg->direction);
  // range
  // cast_shadows
  // spot_inner_angle
  // spot_outer_angle
  // spot_falloff
  // id
  // parent_id
  // intensity
}

bool
ros_gz_interfaces__msg__Light__are_equal(const ros_gz_interfaces__msg__Light * lhs, const ros_gz_interfaces__msg__Light * rhs)
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
  // type
  if (lhs->type != rhs->type) {
    return false;
  }
  // pose
  if (!geometry_msgs__msg__Pose__are_equal(
      &(lhs->pose), &(rhs->pose)))
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
  // attenuation_constant
  if (lhs->attenuation_constant != rhs->attenuation_constant) {
    return false;
  }
  // attenuation_linear
  if (lhs->attenuation_linear != rhs->attenuation_linear) {
    return false;
  }
  // attenuation_quadratic
  if (lhs->attenuation_quadratic != rhs->attenuation_quadratic) {
    return false;
  }
  // direction
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->direction), &(rhs->direction)))
  {
    return false;
  }
  // range
  if (lhs->range != rhs->range) {
    return false;
  }
  // cast_shadows
  if (lhs->cast_shadows != rhs->cast_shadows) {
    return false;
  }
  // spot_inner_angle
  if (lhs->spot_inner_angle != rhs->spot_inner_angle) {
    return false;
  }
  // spot_outer_angle
  if (lhs->spot_outer_angle != rhs->spot_outer_angle) {
    return false;
  }
  // spot_falloff
  if (lhs->spot_falloff != rhs->spot_falloff) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // parent_id
  if (lhs->parent_id != rhs->parent_id) {
    return false;
  }
  // intensity
  if (lhs->intensity != rhs->intensity) {
    return false;
  }
  return true;
}

bool
ros_gz_interfaces__msg__Light__copy(
  const ros_gz_interfaces__msg__Light * input,
  ros_gz_interfaces__msg__Light * output)
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
  // type
  output->type = input->type;
  // pose
  if (!geometry_msgs__msg__Pose__copy(
      &(input->pose), &(output->pose)))
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
  // attenuation_constant
  output->attenuation_constant = input->attenuation_constant;
  // attenuation_linear
  output->attenuation_linear = input->attenuation_linear;
  // attenuation_quadratic
  output->attenuation_quadratic = input->attenuation_quadratic;
  // direction
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->direction), &(output->direction)))
  {
    return false;
  }
  // range
  output->range = input->range;
  // cast_shadows
  output->cast_shadows = input->cast_shadows;
  // spot_inner_angle
  output->spot_inner_angle = input->spot_inner_angle;
  // spot_outer_angle
  output->spot_outer_angle = input->spot_outer_angle;
  // spot_falloff
  output->spot_falloff = input->spot_falloff;
  // id
  output->id = input->id;
  // parent_id
  output->parent_id = input->parent_id;
  // intensity
  output->intensity = input->intensity;
  return true;
}

ros_gz_interfaces__msg__Light *
ros_gz_interfaces__msg__Light__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Light * msg = (ros_gz_interfaces__msg__Light *)allocator.allocate(sizeof(ros_gz_interfaces__msg__Light), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ros_gz_interfaces__msg__Light));
  bool success = ros_gz_interfaces__msg__Light__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ros_gz_interfaces__msg__Light__destroy(ros_gz_interfaces__msg__Light * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ros_gz_interfaces__msg__Light__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ros_gz_interfaces__msg__Light__Sequence__init(ros_gz_interfaces__msg__Light__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Light * data = NULL;

  if (size) {
    data = (ros_gz_interfaces__msg__Light *)allocator.zero_allocate(size, sizeof(ros_gz_interfaces__msg__Light), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ros_gz_interfaces__msg__Light__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ros_gz_interfaces__msg__Light__fini(&data[i - 1]);
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
ros_gz_interfaces__msg__Light__Sequence__fini(ros_gz_interfaces__msg__Light__Sequence * array)
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
      ros_gz_interfaces__msg__Light__fini(&array->data[i]);
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

ros_gz_interfaces__msg__Light__Sequence *
ros_gz_interfaces__msg__Light__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ros_gz_interfaces__msg__Light__Sequence * array = (ros_gz_interfaces__msg__Light__Sequence *)allocator.allocate(sizeof(ros_gz_interfaces__msg__Light__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ros_gz_interfaces__msg__Light__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ros_gz_interfaces__msg__Light__Sequence__destroy(ros_gz_interfaces__msg__Light__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ros_gz_interfaces__msg__Light__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ros_gz_interfaces__msg__Light__Sequence__are_equal(const ros_gz_interfaces__msg__Light__Sequence * lhs, const ros_gz_interfaces__msg__Light__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ros_gz_interfaces__msg__Light__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ros_gz_interfaces__msg__Light__Sequence__copy(
  const ros_gz_interfaces__msg__Light__Sequence * input,
  ros_gz_interfaces__msg__Light__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ros_gz_interfaces__msg__Light);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ros_gz_interfaces__msg__Light * data =
      (ros_gz_interfaces__msg__Light *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ros_gz_interfaces__msg__Light__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ros_gz_interfaces__msg__Light__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ros_gz_interfaces__msg__Light__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
