// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from race_msgs:msg/LocalPathReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__STRUCT_H_
#define RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__STRUCT_H_

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
// Member 'local_goal'
// Member 'points'
// Member 'continuation_points'
#include "geometry_msgs/msg/detail/point__struct.h"

/// Struct defined in msg/LocalPathReference in the package race_msgs.
/**
  * Super's exact, safety-checked polyline from the current path projection to
  * the rolling local goal.  The two counters bind this reference to one global
  * path transaction and one local-goal update.
 */
typedef struct race_msgs__msg__LocalPathReference
{
  std_msgs__msg__Header header;
  uint64_t global_path_id;
  uint64_t local_goal_seq;
  geometry_msgs__msg__Point local_goal;
  geometry_msgs__msg__Point__Sequence points;
  /// Safety-checked continuation of the same global polyline after local_goal.
  /// Normal EGO planning uses only points; recovery uses continuation_points to
  /// choose a rejoin target without extrapolating a single yaw through a corner.
  geometry_msgs__msg__Point__Sequence continuation_points;
  double arc_length;
  double minimum_clearance;
} race_msgs__msg__LocalPathReference;

// Struct for a sequence of race_msgs__msg__LocalPathReference.
typedef struct race_msgs__msg__LocalPathReference__Sequence
{
  race_msgs__msg__LocalPathReference * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} race_msgs__msg__LocalPathReference__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RACE_MSGS__MSG__DETAIL__LOCAL_PATH_REFERENCE__STRUCT_H_
