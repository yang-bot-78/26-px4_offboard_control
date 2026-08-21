// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
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
#include "quadrotor_msgs/msg/detail/aux_command__struct.h"
#include "quadrotor_msgs/msg/detail/aux_command__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool quadrotor_msgs__msg__aux_command__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("quadrotor_msgs.msg._aux_command.AuxCommand", full_classname_dest, 42) == 0);
  }
  quadrotor_msgs__msg__AuxCommand * ros_message = _ros_message;
  {  // current_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "current_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->current_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // kf_correction
    PyObject * field = PyObject_GetAttrString(_pymsg, "kf_correction");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->kf_correction = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // angle_corrections
    PyObject * field = PyObject_GetAttrString(_pymsg, "angle_corrections");
    if (!field) {
      return false;
    }
    {
      // TODO(dirk-thomas) use a better way to check the type before casting
      assert(field->ob_type != NULL);
      assert(field->ob_type->tp_name != NULL);
      assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
      PyArrayObject * seq_field = (PyArrayObject *)field;
      Py_INCREF(seq_field);
      assert(PyArray_NDIM(seq_field) == 1);
      assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
      Py_ssize_t size = 2;
      double * dest = ros_message->angle_corrections;
      for (Py_ssize_t i = 0; i < size; ++i) {
        double tmp = *(npy_float64 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(double));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }
  {  // enable_motors
    PyObject * field = PyObject_GetAttrString(_pymsg, "enable_motors");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->enable_motors = (Py_True == field);
    Py_DECREF(field);
  }
  {  // use_external_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "use_external_yaw");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->use_external_yaw = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * quadrotor_msgs__msg__aux_command__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of AuxCommand */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("quadrotor_msgs.msg._aux_command");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "AuxCommand");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  quadrotor_msgs__msg__AuxCommand * ros_message = (quadrotor_msgs__msg__AuxCommand *)raw_ros_message;
  {  // current_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->current_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "current_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // kf_correction
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->kf_correction);
    {
      int rc = PyObject_SetAttrString(_pymessage, "kf_correction", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // angle_corrections
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "angle_corrections");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_FLOAT64);
    assert(sizeof(npy_float64) == sizeof(double));
    npy_float64 * dst = (npy_float64 *)PyArray_GETPTR1(seq_field, 0);
    double * src = &(ros_message->angle_corrections[0]);
    memcpy(dst, src, 2 * sizeof(double));
    Py_DECREF(field);
  }
  {  // enable_motors
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->enable_motors ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "enable_motors", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // use_external_yaw
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->use_external_yaw ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "use_external_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
