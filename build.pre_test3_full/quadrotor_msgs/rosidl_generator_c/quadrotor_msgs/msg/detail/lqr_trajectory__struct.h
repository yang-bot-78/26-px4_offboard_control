// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/LQRTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'ACTION_ADD'.
/**
  * the action command for trajectory server.
 */
enum
{
  quadrotor_msgs__msg__LQRTrajectory__ACTION_ADD = 1ul
};

/// Constant 'ACTION_ABORT'.
enum
{
  quadrotor_msgs__msg__LQRTrajectory__ACTION_ABORT = 2ul
};

/// Constant 'ACTION_WARN_START'.
enum
{
  quadrotor_msgs__msg__LQRTrajectory__ACTION_WARN_START = 3ul
};

/// Constant 'ACTION_WARN_FINAL'.
enum
{
  quadrotor_msgs__msg__LQRTrajectory__ACTION_WARN_FINAL = 4ul
};

/// Constant 'ACTION_WARN_IMPOSSIBLE'.
enum
{
  quadrotor_msgs__msg__LQRTrajectory__ACTION_WARN_IMPOSSIBLE = 5ul
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'debug_info'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/LQRTrajectory in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__LQRTrajectory
{
  std_msgs__msg__Header header;
  /// the trajectory id, starts from "1".
  uint32_t trajectory_id;
  uint32_t action;
  /// the weight coefficient of the control effort
  double r;
  /// the yaw command
  double start_yaw;
  double final_yaw;
  /// the initial and final state
  double s0[6];
  double ut[3];
  double sf[6];
  /// the optimal arrival time
  double t_f;
  rosidl_runtime_c__String debug_info;
} quadrotor_msgs__msg__LQRTrajectory;

// Struct for a sequence of quadrotor_msgs__msg__LQRTrajectory.
typedef struct quadrotor_msgs__msg__LQRTrajectory__Sequence
{
  quadrotor_msgs__msg__LQRTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__LQRTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__LQR_TRAJECTORY__STRUCT_H_
