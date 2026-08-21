// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_H_

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
  quadrotor_msgs__msg__PolynomialTrajectory__ACTION_ADD = 1ul
};

/// Constant 'ACTION_ABORT'.
enum
{
  quadrotor_msgs__msg__PolynomialTrajectory__ACTION_ABORT = 2ul
};

/// Constant 'ACTION_WARN_START'.
enum
{
  quadrotor_msgs__msg__PolynomialTrajectory__ACTION_WARN_START = 3ul
};

/// Constant 'ACTION_WARN_FINAL'.
enum
{
  quadrotor_msgs__msg__PolynomialTrajectory__ACTION_WARN_FINAL = 4ul
};

/// Constant 'ACTION_WARN_IMPOSSIBLE'.
enum
{
  quadrotor_msgs__msg__PolynomialTrajectory__ACTION_WARN_IMPOSSIBLE = 5ul
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'coef_x'
// Member 'coef_y'
// Member 'coef_z'
// Member 'time'
// Member 'order'
#include "rosidl_runtime_c/primitives_sequence.h"
// Member 'debug_info'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/PolynomialTrajectory in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__PolynomialTrajectory
{
  std_msgs__msg__Header header;
  /// the trajectory id, starts from "1".
  uint32_t trajectory_id;
  uint32_t action;
  /// the order of trajectory.
  uint32_t num_order;
  uint32_t num_segment;
  /// the polynomial coecfficients of the trajectory.
  double start_yaw;
  double final_yaw;
  rosidl_runtime_c__double__Sequence coef_x;
  rosidl_runtime_c__double__Sequence coef_y;
  rosidl_runtime_c__double__Sequence coef_z;
  rosidl_runtime_c__double__Sequence time;
  double mag_coeff;
  rosidl_runtime_c__uint32__Sequence order;
  rosidl_runtime_c__String debug_info;
} quadrotor_msgs__msg__PolynomialTrajectory;

// Struct for a sequence of quadrotor_msgs__msg__PolynomialTrajectory.
typedef struct quadrotor_msgs__msg__PolynomialTrajectory__Sequence
{
  quadrotor_msgs__msg__PolynomialTrajectory * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__PolynomialTrajectory__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__POLYNOMIAL_TRAJECTORY__STRUCT_H_
