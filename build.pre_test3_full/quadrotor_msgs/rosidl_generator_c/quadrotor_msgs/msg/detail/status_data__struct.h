// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/StatusData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__STRUCT_H_

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

/// Struct defined in msg/StatusData in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__StatusData
{
  std_msgs__msg__Header header;
  uint16_t loop_rate;
  double voltage;
  uint8_t seq;
} quadrotor_msgs__msg__StatusData;

// Struct for a sequence of quadrotor_msgs__msg__StatusData.
typedef struct quadrotor_msgs__msg__StatusData__Sequence
{
  quadrotor_msgs__msg__StatusData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__StatusData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__STATUS_DATA__STRUCT_H_
