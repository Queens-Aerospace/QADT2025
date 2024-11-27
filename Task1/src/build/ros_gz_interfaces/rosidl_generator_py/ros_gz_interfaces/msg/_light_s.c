// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/Light.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "ros_gz_interfaces/msg/detail/light__struct.h"
#include "ros_gz_interfaces/msg/detail/light__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__light__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[35];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("ros_gz_interfaces.msg._light.Light", full_classname_dest, 34) == 0);
  }
  ros_gz_interfaces__msg__Light * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // name
    PyObject * field = PyObject_GetAttrString(_pymsg, "name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // type
    PyObject * field = PyObject_GetAttrString(_pymsg, "type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // pose
    PyObject * field = PyObject_GetAttrString(_pymsg, "pose");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__pose__convert_from_py(field, &ros_message->pose)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // diffuse
    PyObject * field = PyObject_GetAttrString(_pymsg, "diffuse");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__color_rgba__convert_from_py(field, &ros_message->diffuse)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // specular
    PyObject * field = PyObject_GetAttrString(_pymsg, "specular");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__color_rgba__convert_from_py(field, &ros_message->specular)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // attenuation_constant
    PyObject * field = PyObject_GetAttrString(_pymsg, "attenuation_constant");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->attenuation_constant = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // attenuation_linear
    PyObject * field = PyObject_GetAttrString(_pymsg, "attenuation_linear");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->attenuation_linear = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // attenuation_quadratic
    PyObject * field = PyObject_GetAttrString(_pymsg, "attenuation_quadratic");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->attenuation_quadratic = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // direction
    PyObject * field = PyObject_GetAttrString(_pymsg, "direction");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->direction)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // range
    PyObject * field = PyObject_GetAttrString(_pymsg, "range");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->range = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // cast_shadows
    PyObject * field = PyObject_GetAttrString(_pymsg, "cast_shadows");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->cast_shadows = (Py_True == field);
    Py_DECREF(field);
  }
  {  // spot_inner_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "spot_inner_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->spot_inner_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // spot_outer_angle
    PyObject * field = PyObject_GetAttrString(_pymsg, "spot_outer_angle");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->spot_outer_angle = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // spot_falloff
    PyObject * field = PyObject_GetAttrString(_pymsg, "spot_falloff");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->spot_falloff = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // parent_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "parent_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->parent_id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // intensity
    PyObject * field = PyObject_GetAttrString(_pymsg, "intensity");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->intensity = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__light__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Light */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._light");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Light");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__Light * ros_message = (ros_gz_interfaces__msg__Light *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->name.data,
      strlen(ros_message->name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // type
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->type);
    {
      int rc = PyObject_SetAttrString(_pymessage, "type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pose
    PyObject * field = NULL;
    field = geometry_msgs__msg__pose__convert_to_py(&ros_message->pose);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "pose", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // diffuse
    PyObject * field = NULL;
    field = std_msgs__msg__color_rgba__convert_to_py(&ros_message->diffuse);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "diffuse", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // specular
    PyObject * field = NULL;
    field = std_msgs__msg__color_rgba__convert_to_py(&ros_message->specular);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "specular", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // attenuation_constant
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->attenuation_constant);
    {
      int rc = PyObject_SetAttrString(_pymessage, "attenuation_constant", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // attenuation_linear
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->attenuation_linear);
    {
      int rc = PyObject_SetAttrString(_pymessage, "attenuation_linear", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // attenuation_quadratic
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->attenuation_quadratic);
    {
      int rc = PyObject_SetAttrString(_pymessage, "attenuation_quadratic", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // direction
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->direction);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "direction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // range
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->range);
    {
      int rc = PyObject_SetAttrString(_pymessage, "range", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // cast_shadows
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->cast_shadows ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "cast_shadows", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // spot_inner_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->spot_inner_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "spot_inner_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // spot_outer_angle
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->spot_outer_angle);
    {
      int rc = PyObject_SetAttrString(_pymessage, "spot_outer_angle", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // spot_falloff
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->spot_falloff);
    {
      int rc = PyObject_SetAttrString(_pymessage, "spot_falloff", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // parent_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->parent_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "parent_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // intensity
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->intensity);
    {
      int rc = PyObject_SetAttrString(_pymessage, "intensity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
