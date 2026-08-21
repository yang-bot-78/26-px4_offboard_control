// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/Gains.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__GAINS__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__GAINS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Gains in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__Gains
{
  double kp;
  double kd;
  double kp_yaw;
  double kd_yaw;
} quadrotor_msgs__msg__Gains;

// Struct for a sequence of quadrotor_msgs__msg__Gains.
typedef struct quadrotor_msgs__msg__Gains__Sequence
{
  quadrotor_msgs__msg__Gains * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__Gains__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__GAINS__STRUCT_H_
