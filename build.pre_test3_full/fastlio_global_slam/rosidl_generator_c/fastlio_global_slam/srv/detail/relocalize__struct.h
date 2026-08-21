// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fastlio_global_slam:srv/Relocalize.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__STRUCT_H_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in srv/Relocalize in the package fastlio_global_slam.
typedef struct fastlio_global_slam__srv__Relocalize_Request
{
  bool use_latest_scan;
} fastlio_global_slam__srv__Relocalize_Request;

// Struct for a sequence of fastlio_global_slam__srv__Relocalize_Request.
typedef struct fastlio_global_slam__srv__Relocalize_Request__Sequence
{
  fastlio_global_slam__srv__Relocalize_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fastlio_global_slam__srv__Relocalize_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"
// Member 'estimated_pose'
#include "geometry_msgs/msg/detail/pose__struct.h"

/// Struct defined in srv/Relocalize in the package fastlio_global_slam.
typedef struct fastlio_global_slam__srv__Relocalize_Response
{
  bool success;
  rosidl_runtime_c__String message;
  geometry_msgs__msg__Pose estimated_pose;
} fastlio_global_slam__srv__Relocalize_Response;

// Struct for a sequence of fastlio_global_slam__srv__Relocalize_Response.
typedef struct fastlio_global_slam__srv__Relocalize_Response__Sequence
{
  fastlio_global_slam__srv__Relocalize_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fastlio_global_slam__srv__Relocalize_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__STRUCT_H_
