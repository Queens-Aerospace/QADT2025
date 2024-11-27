// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from ros_gz_interfaces:msg/EntityFactory.idl
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
#include "ros_gz_interfaces/msg/detail/entity_factory__struct.h"
#include "ros_gz_interfaces/msg/detail/entity_factory__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool geometry_msgs__msg__pose__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * geometry_msgs__msg__pose__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool ros_gz_interfaces__msg__entity_factory__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("ros_gz_interfaces.msg._entity_factory.EntityFactory", full_classname_dest, 51) == 0);
  }
  ros_gz_interfaces__msg__EntityFactory * ros_message = _ros_message;
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
  {  // allow_renaming
    PyObject * field = PyObject_GetAttrString(_pymsg, "allow_renaming");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->allow_renaming = (Py_True == field);
    Py_DECREF(field);
  }
  {  // sdf
    PyObject * field = PyObject_GetAttrString(_pymsg, "sdf");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->sdf, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // sdf_filename
    PyObject * field = PyObject_GetAttrString(_pymsg, "sdf_filename");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->sdf_filename, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // clone_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "clone_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->clone_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
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
  {  // relative_to
    PyObject * field = PyObject_GetAttrString(_pymsg, "relative_to");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->relative_to, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * ros_gz_interfaces__msg__entity_factory__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of EntityFactory */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("ros_gz_interfaces.msg._entity_factory");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "EntityFactory");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  ros_gz_interfaces__msg__EntityFactory * ros_message = (ros_gz_interfaces__msg__EntityFactory *)raw_ros_message;
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
  {  // allow_renaming
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->allow_renaming ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "allow_renaming", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sdf
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->sdf.data,
      strlen(ros_message->sdf.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "sdf", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sdf_filename
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->sdf_filename.data,
      strlen(ros_message->sdf_filename.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "sdf_filename", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // clone_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->clone_name.data,
      strlen(ros_message->clone_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "clone_name", field);
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
  {  // relative_to
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->relative_to.data,
      strlen(ros_message->relative_to.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "relative_to", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
