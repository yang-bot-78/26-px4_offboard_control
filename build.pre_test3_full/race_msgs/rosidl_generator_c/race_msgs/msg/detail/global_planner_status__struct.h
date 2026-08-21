// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from race_msgs:msg/GlobalPlannerStatus.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__STRUCT_H_
#define RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__STRUCT_H_

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
// Member 'mode'
// Member 'reason'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/GlobalPlannerStatus in the package race_msgs.
/**
  * Structured final-goal contract from the global planner to Offboard.
 */
typedef struct race_msgs__msg__GlobalPlannerStatus
{
  std_msgs__msg__Header header;
  uint64_t global_goal_id;
  uint64_t global_path_id;
  uint64_t local_goal_seq;
  bool goal_active;
  bool final_goal_reached;
  double distance_to_final;
  double horizontal_speed;
  rosidl_runtime_c__String mode;
  rosidl_runtime_c__String reason;
} race_msgs__msg__GlobalPlannerStatus;

// Struct for a sequence of race_msgs__msg__GlobalPlannerStatus.
typedef struct race_msgs__msg__GlobalPlannerStatus__Sequence
{
  race_msgs__msg__GlobalPlannerStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} race_msgs__msg__GlobalPlannerStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RACE_MSGS__MSG__DETAIL__GLOBAL_PLANNER_STATUS__STRUCT_H_
