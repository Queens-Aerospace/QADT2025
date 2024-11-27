// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/WorldReset.idl
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
#include "ros_gz_interfaces/msg/detail/world_reset__struct.h"
#include "ros_gz_interfaces/msg/detail/world_reset__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__world_reset__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[46];
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
    assert(strncmp("ros_gz_interfaces.msg._world_reset.WorldReset", full_classname_dest, 45) == 0);
  }
  ros_gz_interfaces__msg__WorldReset * ros_message = _ros_message;
  {  // all
    PyObject * field = PyObject_GetAttrString(_pymsg, "all");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->all = (Py_True == field);
    Py_DECREF(field);
  }
  {  // time_only
    PyObject * field = PyObject_GetAttrString(_pymsg, "time_only");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->time_only = (Py_True == field);
    Py_DECREF(field);
  }
  {  // model_only
    PyObject * field = PyObject_GetAttrString(_pymsg, "model_only");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->model_only = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__world_reset__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of WorldReset */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._world_reset");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "WorldReset");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__WorldReset * ros_message = (ros_gz_interfaces__msg__WorldReset *)raw_ros_message;
  {  // all
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->all ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "all", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time_only
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->time_only ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time_only", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // model_only
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->model_only ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "model_only", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
