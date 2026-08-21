// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "race_msgs/msg/detail/flight_altitude_reference__rosidl_typesupport_introspection_c.h"
#include "race_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "race_msgs/msg/detail/flight_altitude_reference__functions.h"
#include "race_msgs/msg/detail/flight_altitude_reference__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  race_msgs__msg__FlightAltitudeReference__init(message_memory);
}

void race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_fini_function(void * message_memory)
{
  race_msgs__msg__FlightAltitudeReference__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_member_array[10] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "flight_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, flight_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_agl_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, target_agl_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "min_agl_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, min_agl_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "max_agl_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, max_agl_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ground_z_local_ned",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, ground_z_local_ned),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_z_local_ned",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, target_z_local_ned),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "ground_z_map",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, ground_z_map),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "target_z_map",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__FlightAltitudeReference, target_z_map),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_members = {
  "race_msgs__msg",  // message namespace
  "FlightAltitudeReference",  // message name
  10,  // number of fields
  sizeof(race_msgs__msg__FlightAltitudeReference),
  race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_member_array,  // message members
  race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_init_function,  // function to initialize message memory (memory has to be allocated)
  race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_type_support_handle = {
  0,
  &race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_race_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, race_msgs, msg, FlightAltitudeReference)() {
  race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_type_support_handle.typesupport_identifier) {
    race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &race_msgs__msg__FlightAltitudeReference__rosidl_typesupport_introspection_c__FlightAltitudeReference_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
