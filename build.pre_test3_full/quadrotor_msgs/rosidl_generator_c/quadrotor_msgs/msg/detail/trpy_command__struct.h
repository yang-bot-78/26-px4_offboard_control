// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/TRPYCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__TRPY_COMMAND__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__TRPY_COMMAND__STRUCT_H_

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
// Member 'aux'
#include "quadrotor_msgs/msg/detail/aux_command__struct.h"

/// Struct defined in msg/TRPYCommand in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__TRPYCommand
{
  std_msgs__msg__Header header;
  float thrust;
  float roll;
  float pitch;
  float yaw;
  quadrotor_msgs__msg__AuxCommand aux;
} quadrotor_msgs__msg__TRPYCommand;

// Struct for a sequence of quadrotor_msgs__msg__TRPYCommand.
typedef struct quadrotor_msgs__msg__TRPYCommand__Sequence
{
  quadrotor_msgs__msg__TRPYCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__TRPYCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__TRPY_COMMAND__STRUCT_H_
