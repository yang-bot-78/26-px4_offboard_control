// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from quadrotor_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "quadrotor_msgs/msg/detail/polynomial_trajectory__rosidl_typesupport_introspection_c.h"
#include "quadrotor_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "quadrotor_msgs/msg/detail/polynomial_trajectory__functions.h"
#include "quadrotor_msgs/msg/detail/polynomial_trajectory__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `coef_x`
// Member `coef_y`
// Member `coef_z`
// Member `time`
// Member `order`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `debug_info`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  quadrotor_msgs__msg__PolynomialTrajectory__init(message_memory);
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_fini_function(void * message_memory)
{
  quadrotor_msgs__msg__PolynomialTrajectory__fini(message_memory);
}

size_t quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__coef_x(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_x(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_x(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__coef_x(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_x(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__coef_x(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_x(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__coef_x(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__coef_y(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_y(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_y(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__coef_y(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_y(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__coef_y(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_y(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__coef_y(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__coef_z(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_z(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_z(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__coef_z(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_z(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__coef_z(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_z(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__coef_z(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__time(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__time(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__time(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__time(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__time(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__time(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__time(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__time(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__order(
  const void * untyped_member)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return member->size;
}

const void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__order(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__uint32__Sequence * member =
    (const rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void * quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__order(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  return &member->data[index];
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__order(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const uint32_t * item =
    ((const uint32_t *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__order(untyped_member, index));
  uint32_t * value =
    (uint32_t *)(untyped_value);
  *value = *item;
}

void quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__order(
  void * untyped_member, size_t index, const void * untyped_value)
{
  uint32_t * item =
    ((uint32_t *)
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__order(untyped_member, index));
  const uint32_t * value =
    (const uint32_t *)(untyped_value);
  *item = *value;
}

bool quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__order(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__uint32__Sequence * member =
    (rosidl_runtime_c__uint32__Sequence *)(untyped_member);
  rosidl_runtime_c__uint32__Sequence__fini(member);
  return rosidl_runtime_c__uint32__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_member_array[14] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "trajectory_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, trajectory_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "action",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, action),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_order",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, num_order),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "num_segment",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, num_segment),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "start_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, start_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "final_yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, final_yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "coef_x",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, coef_x),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__coef_x,  // size() function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_x,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_x,  // get(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__coef_x,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__coef_x,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__coef_x  // resize(index) function pointer
  },
  {
    "coef_y",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, coef_y),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__coef_y,  // size() function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_y,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_y,  // get(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__coef_y,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__coef_y,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__coef_y  // resize(index) function pointer
  },
  {
    "coef_z",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, coef_z),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__coef_z,  // size() function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__coef_z,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__coef_z,  // get(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__coef_z,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__coef_z,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__coef_z  // resize(index) function pointer
  },
  {
    "time",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, time),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__time,  // size() function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__time,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__time,  // get(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__time,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__time,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__time  // resize(index) function pointer
  },
  {
    "mag_coeff",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, mag_coeff),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "order",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, order),  // bytes offset in struct
    NULL,  // default value
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__size_function__PolynomialTrajectory__order,  // size() function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_const_function__PolynomialTrajectory__order,  // get_const(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__get_function__PolynomialTrajectory__order,  // get(index) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__fetch_function__PolynomialTrajectory__order,  // fetch(index, &value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__assign_function__PolynomialTrajectory__order,  // assign(index, value) function pointer
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__resize_function__PolynomialTrajectory__order  // resize(index) function pointer
  },
  {
    "debug_info",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(quadrotor_msgs__msg__PolynomialTrajectory, debug_info),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_members = {
  "quadrotor_msgs__msg",  // message namespace
  "PolynomialTrajectory",  // message name
  14,  // number of fields
  sizeof(quadrotor_msgs__msg__PolynomialTrajectory),
  quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_member_array,  // message members
  quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_init_function,  // function to initialize message memory (memory has to be allocated)
  quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_type_support_handle = {
  0,
  &quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_quadrotor_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, quadrotor_msgs, msg, PolynomialTrajectory)() {
  quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_type_support_handle.typesupport_identifier) {
    quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &quadrotor_msgs__msg__PolynomialTrajectory__rosidl_typesupport_introspection_c__PolynomialTrajectory_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
