// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from quadrotor_msgs:msg/PPROutputData.idl
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
#include "quadrotor_msgs/msg/detail/ppr_output_data__struct.h"
#include "quadrotor_msgs/msg/detail/ppr_output_data__functions.h"

#include "rosidl_runtime_c/primitives_sequence.h"
#include "rosidl_runtime_c/primitives_sequence_functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool quadrotor_msgs__msg__ppr_output_data__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("quadrotor_msgs.msg._ppr_output_data.PPROutputData", full_classname_dest, 49) == 0);
  }
  quadrotor_msgs__msg__PPROutputData * ros_message = _ros_message;
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
  {  // quad_time
    PyObject * field = PyObject_GetAttrString(_pymsg, "quad_time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->quad_time = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // des_thrust
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_thrust");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_thrust = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_roll = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_pitch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // des_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "des_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->des_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_roll
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_roll");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_roll = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_pitch
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_pitch");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_pitch = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_yaw
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_yaw");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_yaw = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_angvel_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_angvel_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_angvel_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_angvel_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_angvel_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_angvel_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_angvel_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_angvel_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_angvel_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_acc_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_acc_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_acc_x = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_acc_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_acc_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_acc_y = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // est_acc_z
    PyObject * field = PyObject_GetAttrString(_pymsg, "est_acc_z");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->est_acc_z = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pwm
    PyObject * field = PyObject_GetAttrString(_pymsg, "pwm");
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
      assert(PyArray_TYPE(seq_field) == NPY_UINT16);
      Py_ssize_t size = 4;
      uint16_t * dest = ros_message->pwm;
      for (Py_ssize_t i = 0; i < size; ++i) {
        uint16_t tmp = *(npy_uint16 *)PyArray_GETPTR1(seq_field, i);
        memcpy(&dest[i], &tmp, sizeof(uint16_t));
      }
      Py_DECREF(seq_field);
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * quadrotor_msgs__msg__ppr_output_data__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of PPROutputData */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("quadrotor_msgs.msg._ppr_output_data");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "PPROutputData");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  quadrotor_msgs__msg__PPROutputData * ros_message = (quadrotor_msgs__msg__PPROutputData *)raw_ros_message;
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
  {  // quad_time
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->quad_time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "quad_time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_thrust
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_thrust);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_thrust", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // des_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->des_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "des_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_roll
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_roll);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_roll", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_pitch
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_pitch);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_pitch", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_yaw
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_yaw);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_yaw", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_angvel_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_angvel_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_angvel_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_angvel_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_angvel_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_angvel_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_angvel_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_angvel_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_angvel_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_acc_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_acc_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_acc_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_acc_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_acc_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_acc_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // est_acc_z
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->est_acc_z);
    {
      int rc = PyObject_SetAttrString(_pymessage, "est_acc_z", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pwm
    PyObject * field = NULL;
    field = PyObject_GetAttrString(_pymessage, "pwm");
    if (!field) {
      return NULL;
    }
    assert(field->ob_type != NULL);
    assert(field->ob_type->tp_name != NULL);
    assert(strcmp(field->ob_type->tp_name, "numpy.ndarray") == 0);
    PyArrayObject * seq_field = (PyArrayObject *)field;
    assert(PyArray_NDIM(seq_field) == 1);
    assert(PyArray_TYPE(seq_field) == NPY_UINT16);
    assert(sizeof(npy_uint16) == sizeof(uint16_t));
    npy_uint16 * dst = (npy_uint16 *)PyArray_GETPTR1(seq_field, 0);
    uint16_t * src = &(ros_message->pwm[0]);
    memcpy(dst, src, 4 * sizeof(uint16_t));
    Py_DECREF(field);
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
