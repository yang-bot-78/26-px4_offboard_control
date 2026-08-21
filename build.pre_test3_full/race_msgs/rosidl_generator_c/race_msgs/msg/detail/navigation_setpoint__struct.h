// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from race_msgs:msg/NavigationSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__STRUCT_H_
#define RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__STRUCT_H_

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
// Member 'position'
#include "geometry_msgs/msg/detail/point__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/NavigationSetpoint in the package race_msgs.
/**
  * Planner-to-controller contract. All values use header.frame_id.
 */
typedef struct race_msgs__msg__NavigationSetpoint
{
  std_msgs__msg__Header header;
  geometry_msgs__msg__Point position;
  /// Optional path-tangent velocity feed-forward.
  bool velocity_valid;
  geometry_msgs__msg__Vector3 velocity;
  /// Heading and optional heading-rate feed-forward in radians and radians/second.
  double yaw;
  bool yaw_rate_valid;
  double yaw_rate;
} race_msgs__msg__NavigationSetpoint;

// Struct for a sequence of race_msgs__msg__NavigationSetpoint.
typedef struct race_msgs__msg__NavigationSetpoint__Sequence
{
  race_msgs__msg__NavigationSetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} race_msgs__msg__NavigationSetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__STRUCT_H_
