// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/Corrections.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/Corrections in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__Corrections
{
  double kf_correction;
  double angle_corrections[2];
} quadrotor_msgs__msg__Corrections;

// Struct for a sequence of quadrotor_msgs__msg__Corrections.
typedef struct quadrotor_msgs__msg__Corrections__Sequence
{
  quadrotor_msgs__msg__Corrections * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__Corrections__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__CORRECTIONS__STRUCT_H_
