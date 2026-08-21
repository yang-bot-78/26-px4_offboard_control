// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from race_msgs:msg/GlobalPlannerStatus.idl
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
#include "race_msgs/msg/detail/global_planner_status__struct.h"
#include "race_msgs/msg/detail/global_planner_status__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool race_msgs__msg__global_planner_status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[57];
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
    assert(strncmp("race_msgs.msg._global_planner_status.GlobalPlannerStatus", full_classname_dest, 56) == 0);
  }
  race_msgs__msg__GlobalPlannerStatus * ros_message = _ros_message;
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
  {  // global_goal_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "global_goal_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->global_goal_id = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // global_path_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "global_path_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->global_path_id = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // local_goal_seq
    PyObject * field = PyObject_GetAttrString(_pymsg, "local_goal_seq");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->local_goal_seq = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // goal_active
    PyObject * field = PyObject_GetAttrString(_pymsg, "goal_active");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->goal_active = (Py_True == field);
    Py_DECREF(field);
  }
  {  // final_goal_reached
    PyObject * field = PyObject_GetAttrString(_pymsg, "final_goal_reached");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->final_goal_reached = (Py_True == field);
    Py_DECREF(field);
  }
  {  // distance_to_final
    PyObject * field = PyObject_GetAttrString(_pymsg, "distance_to_final");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->distance_to_final = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // horizontal_speed
    PyObject * field = PyObject_GetAttrString(_pymsg, "horizontal_speed");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->horizontal_speed = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // mode
    PyObject * field = PyObject_GetAttrString(_pymsg, "mode");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->mode, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // reason
    PyObject * field = PyObject_GetAttrString(_pymsg, "reason");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->reason, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * race_msgs__msg__global_planner_status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of GlobalPlannerStatus */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("race_msgs.msg._global_planner_status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "GlobalPlannerStatus");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  race_msgs__msg__GlobalPlannerStatus * ros_message = (race_msgs__msg__GlobalPlannerStatus *)raw_ros_message;
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
  {  // global_goal_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->global_goal_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "global_goal_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // global_path_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->global_path_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "global_path_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // local_goal_seq
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->local_goal_seq);
    {
      int rc = PyObject_SetAttrString(_pymessage, "local_goal_seq", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // goal_active
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->goal_active ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "goal_active", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // final_goal_reached
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->final_goal_reached ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "final_goal_reached", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // distance_to_final
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->distance_to_final);
    {
      int rc = PyObject_SetAttrString(_pymessage, "distance_to_final", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // horizontal_speed
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->horizontal_speed);
    {
      int rc = PyObject_SetAttrString(_pymessage, "horizontal_speed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // mode
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->mode.data,
      strlen(ros_message->mode.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "mode", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reason
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->reason.data,
      strlen(ros_message->reason.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "reason", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
