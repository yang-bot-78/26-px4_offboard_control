// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__STRUCT_H_
#define RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__STRUCT_H_

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

/// Struct defined in msg/FlightAltitudeReference in the package race_msgs.
typedef struct race_msgs__msg__FlightAltitudeReference
{
  std_msgs__msg__Header header;
  /// Monotonic identifier assigned by the Offboard authority for each takeoff.
  uint64_t flight_id;
  bool valid;
  /// All configured vertical limits are heights above the locked takeoff ground.
  double target_agl_m;
  double min_agl_m;
  double max_agl_m;
  /// PX4 local position uses NED (positive down).
  double ground_z_local_ned;
  double target_z_local_ned;
  /// Navigation and EGO use map ENU (positive up).
  double ground_z_map;
  double target_z_map;
} race_msgs__msg__FlightAltitudeReference;

// Struct for a sequence of race_msgs__msg__FlightAltitudeReference.
typedef struct race_msgs__msg__FlightAltitudeReference__Sequence
{
  race_msgs__msg__FlightAltitudeReference * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} race_msgs__msg__FlightAltitudeReference__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__STRUCT_H_
