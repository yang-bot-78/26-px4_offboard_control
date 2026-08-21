// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from quadrotor_msgs:msg/OutputData.idl
// generated code does not contain a copyright notice

#ifndef QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__STRUCT_H_
#define QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__STRUCT_H_

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
// Member 'orientation'
#include "geometry_msgs/msg/detail/quaternion__struct.h"
// Member 'angular_velocity'
// Member 'linear_acceleration'
// Member 'magnetic_field'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/OutputData in the package quadrotor_msgs.
typedef struct quadrotor_msgs__msg__OutputData
{
  std_msgs__msg__Header header;
  uint16_t loop_rate;
  double voltage;
  geometry_msgs__msg__Quaternion orientation;
  geometry_msgs__msg__Vector3 angular_velocity;
  geometry_msgs__msg__Vector3 linear_acceleration;
  double pressure_dheight;
  double pressure_height;
  geometry_msgs__msg__Vector3 magnetic_field;
  uint8_t radio_channel[8];
  /// uint8 motor_rpm
  uint8_t seq;
} quadrotor_msgs__msg__OutputData;

// Struct for a sequence of quadrotor_msgs__msg__OutputData.
typedef struct quadrotor_msgs__msg__OutputData__Sequence
{
  quadrotor_msgs__msg__OutputData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} quadrotor_msgs__msg__OutputData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // QUADROTOR_MSGS__MSG__DETAIL__OUTPUT_DATA__STRUCT_H_
