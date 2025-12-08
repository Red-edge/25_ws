// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
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
#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__struct.h"
#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool tb4_autonav_interfaces__msg__yolo_target_bias__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[60];
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
    assert(strncmp("tb4_autonav_interfaces.msg._yolo_target_bias.YoloTargetBias", full_classname_dest, 59) == 0);
  }
  tb4_autonav_interfaces__msg__YoloTargetBias * ros_message = _ros_message;
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
  {  // has_target
    PyObject * field = PyObject_GetAttrString(_pymsg, "has_target");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->has_target = (Py_True == field);
    Py_DECREF(field);
  }
  {  // type
    PyObject * field = PyObject_GetAttrString(_pymsg, "type");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->type, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // distance_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_m = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // u_norm
    PyObject * field = PyObject_GetAttrString(_pymsg, "u_norm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->u_norm = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // v_norm
    PyObject * field = PyObject_GetAttrString(_pymsg, "v_norm");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->v_norm = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * tb4_autonav_interfaces__msg__yolo_target_bias__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of YoloTargetBias */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("tb4_autonav_interfaces.msg._yolo_target_bias");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "YoloTargetBias");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  tb4_autonav_interfaces__msg__YoloTargetBias * ros_message = (tb4_autonav_interfaces__msg__YoloTargetBias *)raw_ros_message;
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
  {  // has_target
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->has_target ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "has_target", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // type
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->type.data,
      strlen(ros_message->type.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "type", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // u_norm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->u_norm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "u_norm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // v_norm
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->v_norm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "v_norm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
