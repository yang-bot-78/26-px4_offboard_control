// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/SO3Command.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'force'
#include "geometry_msgs/msg/detail/vector3__struct.h"
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'aux'
#include "quadrotor_msgs/msg/detail/aux_command__struct.h"

/// Struct defined in msg/SO3Command in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__SO3Command
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Vector3 force;
  geometry_msgs__msg__Quaternion orientation;
  double kr[3];
  double kom[3];
  quadrotor_msgs__msg__AuxCommand aux;
} quadrotor_msgs__msg__SO3Command;

// Struct for a sequence of quadrotor_msgs__msg__SO3Command.
typedef struct quadrotor_msgs__msg__SO3Command__Sequence
{
  quadrotor_msgs__msg__SO3Command * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__SO3Command__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__SO3_COMMAND__STRUCT_H_
