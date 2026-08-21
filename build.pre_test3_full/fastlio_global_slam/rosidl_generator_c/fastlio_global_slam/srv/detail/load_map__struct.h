// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from fastlio_global_slam:srv/LoadMap.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__LOAD_MAP__STRUCT_H_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__LOAD_MAP__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'directory'
#include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LoadMap in the package fastlio_global_slam.
typedef struct fastlio_global_slam__srv__LoadMap_Request
{
  rosidl_runtime_c__String directory;
} fastlio_global_slam__srv__LoadMap_Request;

// Struct for a sequence of fastlio_global_slam__srv__LoadMap_Request.
typedef struct fastlio_global_slam__srv__LoadMap_Request__Sequence
{
  fastlio_global_slam__srv__LoadMap_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fastlio_global_slam__srv__LoadMap_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'message'
// already included above
// #include "rosidl_runtime_c/string.h"

/// Struct defined in srv/LoadMap in the package fastlio_global_slam.
typedef struct fastlio_global_slam__srv__LoadMap_Response
{
  bool success;
  rosidl_runtime_c__String message;
} fastlio_global_slam__srv__LoadMap_Response;

// Struct for a sequence of fastlio_global_slam__srv__LoadMap_Response.
typedef struct fastlio_global_slam__srv__LoadMap_Response__Sequence
{
  fastlio_global_slam__srv__LoadMap_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} fastlio_global_slam__srv__LoadMap_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__LOAD_MAP__STRUCT_H_
