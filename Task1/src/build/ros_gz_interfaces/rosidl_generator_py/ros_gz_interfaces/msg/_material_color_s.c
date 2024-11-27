// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/MaterialColor.idl
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
#include "ros_gz_interfaces/msg/detail/material_color__struct.h"
#include "ros_gz_interfaces/msg/detail/material_color__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
bool ros_gz_interfaces__msg__entity__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros_gz_interfaces__msg__entity__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__color_rgba__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__color_rgba__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__material_color__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[52];
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
    assert(strncmp("ros_gz_interfaces.msg._material_color.MaterialColor", full_classname_dest, 51) == 0);
  }
  ros_gz_interfaces__msg__MaterialColor * ros_message = _ros_message;
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
  {  // entity
    PyObject * field = PyObject_GetAttrString(_pymsg, "entity");
    if (!field) {
      return false;
    }
    if (!ros_gz_interfaces__msg__entity__convert_from_py(field, &ros_message->entity)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // ambient
    PyObject * field = PyObject_GetAttrString(_pymsg, "ambient");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__color_rgba__convert_from_py(field, &ros_message->ambient)) {
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
  {  // emissive
    PyObject * field = PyObject_GetAttrString(_pymsg, "emissive");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__color_rgba__convert_from_py(field, &ros_message->emissive)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // shininess
    PyObject * field = PyObject_GetAttrString(_pymsg, "shininess");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->shininess = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // entity_match
    PyObject * field = PyObject_GetAttrString(_pymsg, "entity_match");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->entity_match = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__material_color__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of MaterialColor */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._material_color");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "MaterialColor");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__MaterialColor * ros_message = (ros_gz_interfaces__msg__MaterialColor *)raw_ros_message;
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
  {  // entity
    PyObject * field = NULL;
    field = ros_gz_interfaces__msg__entity__convert_to_py(&ros_message->entity);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "entity", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ambient
    PyObject * field = NULL;
    field = std_msgs__msg__color_rgba__convert_to_py(&ros_message->ambient);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "ambient", field);
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
  {  // emissive
    PyObject * field = NULL;
    field = std_msgs__msg__color_rgba__convert_to_py(&ros_message->emissive);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "emissive", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // shininess
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->shininess);
    {
      int rc = PyObject_SetAttrString(_pymessage, "shininess", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // entity_match
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->entity_match);
    {
      int rc = PyObject_SetAttrString(_pymessage, "entity_match", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
