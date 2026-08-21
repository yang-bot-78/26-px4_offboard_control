// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'STATUS_ODOM_VALID'.
enum
{
  quadrotor_msgs__msg__Odometry__STATUS_ODOM_VALID = 0
};

/// Constant 'STATUS_ODOM_INVALID'.
enum
{
  quadrotor_msgs__msg__Odometry__STATUS_ODOM_INVALID = 1
};

/// Constant 'STATUS_ODOM_LOOPCLOSURE'.
enum
{
  quadrotor_msgs__msg__Odometry__STATUS_ODOM_LOOPCLOSURE = 2
};

// Include directives for member types
// Member 'curodom'
// Member 'kfodom'
#include "nav_msgs/msg/detail/odometry__struct.h"

/// Struct defined in msg/Odometry in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__Odometry
{
  nav_msgs__msg__Odometry curodom;
  nav_msgs__msg__Odometry kfodom;
  uint32_t kfid;
  uint8_t status;
} quadrotor_msgs__msg__Odometry;

// Struct for a sequence of quadrotor_msgs__msg__Odometry.
typedef struct quadrotor_msgs__msg__Odometry__Sequence
{
  quadrotor_msgs__msg__Odometry * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__Odometry__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__ODOMETRY__STRUCT_H_
