// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/TrackVisual.idl
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
#include "ros_gz_interfaces/msg/detail/track_visual__struct.h"
#include "ros_gz_interfaces/msg/detail/track_visual__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__vector3__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__vector3__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__track_visual__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[48];
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
    assert(strncmp("ros_gz_interfaces.msg._track_visual.TrackVisual", full_classname_dest, 47) == 0);
  }
  ros_gz_interfaces__msg__TrackVisual * ros_message = _ros_message;
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
  {  // id
    PyObject * field = PyObject_GetAttrString(_pymsg, "id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->id = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // inherit_orientation
    PyObject * field = PyObject_GetAttrString(_pymsg, "inherit_orientation");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->inherit_orientation = (Py_True == field);
    Py_DECREF(field);
  }
  {  // min_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_dist = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_dist
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_dist");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_dist = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // is_static
    PyObject * field = PyObject_GetAttrString(_pymsg, "is_static");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->is_static = (Py_True == field);
    Py_DECREF(field);
  }
  {  // use_model_frame
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_model_frame");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_model_frame = (Py_True == field);
    Py_DECREF(field);
  }
  {  // xyz
    PyObject * field = PyObject_GetAttrString(_pymsg, "xyz");
    if (!field) {
      return false;
    }
    if (!geometry_msgs__msg__vector3__convert_from_py(field, &ros_message->xyz)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // inherit_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "inherit_yaw");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->inherit_yaw = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__track_visual__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of TrackVisual */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._track_visual");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "TrackVisual");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__TrackVisual * ros_message = (ros_gz_interfaces__msg__TrackVisual *)raw_ros_message;
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
  {  // inherit_orientation
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->inherit_orientation ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "inherit_orientation", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_dist
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_dist);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_dist", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // is_static
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->is_static ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "is_static", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_model_frame
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_model_frame ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_model_frame", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // xyz
    PyObject * field = NULL;
    field = geometry_msgs__msg__vector3__convert_to_py(&ros_message->xyz);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "xyz", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // inherit_yaw
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->inherit_yaw ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "inherit_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
