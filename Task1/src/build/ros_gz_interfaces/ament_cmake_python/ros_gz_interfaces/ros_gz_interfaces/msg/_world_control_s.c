// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/WorldControl.idl
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
#include "ros_gz_interfaces/msg/detail/world_control__struct.h"
#include "ros_gz_interfaces/msg/detail/world_control__functions.h"

bool ros_gz_interfaces__msg__world_reset__convert_from_py(PyObject * _pymsg, void * _ros_message);
PyObject * ros_gz_interfaces__msg__world_reset__convert_to_py(void * raw_ros_message);
ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__time__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__time__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__world_control__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
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
    assert(strncmp("ros_gz_interfaces.msg._world_control.WorldControl", full_classname_dest, 49) == 0);
  }
  ros_gz_interfaces__msg__WorldControl * ros_message = _ros_message;
  {  // pause
    PyObject * field = PyObject_GetAttrString(_pymsg, "pause");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->pause = (Py_True == field);
    Py_DECREF(field);
  }
  {  // step
    PyObject * field = PyObject_GetAttrString(_pymsg, "step");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->step = (Py_True == field);
    Py_DECREF(field);
  }
  {  // multi_step
    PyObject * field = PyObject_GetAttrString(_pymsg, "multi_step");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->multi_step = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // reset
    PyObject * field = PyObject_GetAttrString(_pymsg, "reset");
    if (!field) {
      return false;
    }
    if (!ros_gz_interfaces__msg__world_reset__convert_from_py(field, &ros_message->reset)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // seed
    PyObject * field = PyObject_GetAttrString(_pymsg, "seed");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->seed = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // run_to_sim_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "run_to_sim_time");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__time__convert_from_py(field, &ros_message->run_to_sim_time)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__world_control__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of WorldControl */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._world_control");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "WorldControl");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__WorldControl * ros_message = (ros_gz_interfaces__msg__WorldControl *)raw_ros_message;
  {  // pause
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->pause ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pause", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // step
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->step ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "step", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // multi_step
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->multi_step);
    {
      int rc = PyObject_SetAttrString(_pymessage, "multi_step", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reset
    PyObject * field = NULL;
    field = ros_gz_interfaces__msg__world_reset__convert_to_py(&ros_message->reset);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "reset", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // seed
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->seed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "seed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // run_to_sim_time
    PyObject * field = NULL;
    field = builtin_interfaces__msg__time__convert_to_py(&ros_message->run_to_sim_time);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "run_to_sim_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
