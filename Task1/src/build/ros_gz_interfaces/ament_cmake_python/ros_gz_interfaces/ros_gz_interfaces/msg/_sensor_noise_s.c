// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/SensorNoise.idl
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
#include "ros_gz_interfaces/msg/detail/sensor_noise__struct.h"
#include "ros_gz_interfaces/msg/detail/sensor_noise__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__sensor_noise__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ros_gz_interfaces.msg._sensor_noise.SensorNoise", full_classname_dest, 47) == 0);
  }
  ros_gz_interfaces__msg__SensorNoise * ros_message = _ros_message;
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
  {  // type
    PyObject * field = PyObject_GetAttrString(_pymsg, "type");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->type = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // mean
    PyObject * field = PyObject_GetAttrString(_pymsg, "mean");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->mean = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // stddev
    PyObject * field = PyObject_GetAttrString(_pymsg, "stddev");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->stddev = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_mean
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_mean");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_mean = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // bias_stddev
    PyObject * field = PyObject_GetAttrString(_pymsg, "bias_stddev");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->bias_stddev = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // precision
    PyObject * field = PyObject_GetAttrString(_pymsg, "precision");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->precision = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dynamic_bias_stddev
    PyObject * field = PyObject_GetAttrString(_pymsg, "dynamic_bias_stddev");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dynamic_bias_stddev = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // dynamic_bias_correlation_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "dynamic_bias_correlation_time");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->dynamic_bias_correlation_time = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__sensor_noise__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of SensorNoise */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._sensor_noise");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "SensorNoise");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__SensorNoise * ros_message = (ros_gz_interfaces__msg__SensorNoise *)raw_ros_message;
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
  {  // mean
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->mean);
    {
      int rc = PyObject_SetAttrString(_pymessage, "mean", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // stddev
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->stddev);
    {
      int rc = PyObject_SetAttrString(_pymessage, "stddev", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_mean
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_mean);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_mean", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // bias_stddev
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->bias_stddev);
    {
      int rc = PyObject_SetAttrString(_pymessage, "bias_stddev", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // precision
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->precision);
    {
      int rc = PyObject_SetAttrString(_pymessage, "precision", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dynamic_bias_stddev
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dynamic_bias_stddev);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dynamic_bias_stddev", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dynamic_bias_correlation_time
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->dynamic_bias_correlation_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "dynamic_bias_correlation_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
