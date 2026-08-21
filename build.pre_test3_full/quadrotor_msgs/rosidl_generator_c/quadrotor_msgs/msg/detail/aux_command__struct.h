// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/AuxCommand in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__AuxCommand
{
  double current_yaw;
  double kf_correction;
  /// Trims for roll, pitch
  double angle_corrections[2];
  bool enable_motors;
  bool use_external_yaw;
} quadrotor_msgs__msg__AuxCommand;

// Struct for a sequence of quadrotor_msgs__msg__AuxCommand.
typedef struct quadrotor_msgs__msg__AuxCommand__Sequence
{
  quadrotor_msgs__msg__AuxCommand * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__AuxCommand__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__AUX_COMMAND__STRUCT_H_
