// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/PPROutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__STRUCT_H_

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

/// Struct defined in msg/PPROutputData in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__PPROutputData
{
  std_msgs__msg__Header header;
  uint16_t quad_time;
  double des_thrust;
  double des_roll;
  double des_pitch;
  double des_yaw;
  double est_roll;
  double est_pitch;
  double est_yaw;
  double est_angvel_x;
  double est_angvel_y;
  double est_angvel_z;
  double est_acc_x;
  double est_acc_y;
  double est_acc_z;
  uint16_t pwm[4];
} quadrotor_msgs__msg__PPROutputData;

// Struct for a sequence of quadrotor_msgs__msg__PPROutputData.
typedef struct quadrotor_msgs__msg__PPROutputData__Sequence
{
  quadrotor_msgs__msg__PPROutputData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__PPROutputData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__PPR_OUTPUT_DATA__STRUCT_H_
