// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
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
#include "race_msgs/msg/detail/flight_altitude_reference__struct.h"
#include "race_msgs/msg/detail/flight_altitude_reference__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool race_msgs__msg__flight_altitude_reference__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[65];
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
    assert(strncmp("race_msgs.msg._flight_altitude_reference.FlightAltitudeReference", full_classname_dest, 64) == 0);
  }
  race_msgs__msg__FlightAltitudeReference * ros_message = _ros_message;
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
  {  // flight_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "flight_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->flight_id = PyLong_AsUnsignedLongLong(field);
    Py_DECREF(field);
  }
  {  // valid
    PyObject * field = PyObject_GetAttrString(_pymsg, "valid");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->valid = (Py_True == field);
    Py_DECREF(field);
  }
  {  // target_agl_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_agl_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_agl_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // min_agl_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_agl_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_agl_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // max_agl_m
    PyObject * field = PyObject_GetAttrString(_pymsg, "max_agl_m");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->max_agl_m = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ground_z_local_ned
    PyObject * field = PyObject_GetAttrString(_pymsg, "ground_z_local_ned");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ground_z_local_ned = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_z_local_ned
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_z_local_ned");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_z_local_ned = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // ground_z_map
    PyObject * field = PyObject_GetAttrString(_pymsg, "ground_z_map");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->ground_z_map = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // target_z_map
    PyObject * field = PyObject_GetAttrString(_pymsg, "target_z_map");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->target_z_map = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * race_msgs__msg__flight_altitude_reference__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of FlightAltitudeReference */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("race_msgs.msg._flight_altitude_reference");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "FlightAltitudeReference");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  race_msgs__msg__FlightAltitudeReference * ros_message = (race_msgs__msg__FlightAltitudeReference *)raw_ros_message;
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
  {  // flight_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLongLong(ros_message->flight_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "flight_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // valid
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->valid ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "valid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_agl_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_agl_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_agl_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_agl_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_agl_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_agl_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // max_agl_m
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->max_agl_m);
    {
      int rc = PyObject_SetAttrString(_pymessage, "max_agl_m", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ground_z_local_ned
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ground_z_local_ned);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ground_z_local_ned", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_z_local_ned
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_z_local_ned);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_z_local_ned", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // ground_z_map
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->ground_z_map);
    {
      int rc = PyObject_SetAttrString(_pymessage, "ground_z_map", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // target_z_map
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->target_z_map);
    {
      int rc = PyObject_SetAttrString(_pymessage, "target_z_map", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
