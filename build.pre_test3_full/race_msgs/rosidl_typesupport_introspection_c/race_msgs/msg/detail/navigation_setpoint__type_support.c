// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from race_msgs:msg/NavigationSetpoint.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "race_msgs/msg/detail/navigation_setpoint__rosidl_typesupport_introspection_c.h"
#include "race_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "race_msgs/msg/detail/navigation_setpoint__functions.h"
#include "race_msgs/msg/detail/navigation_setpoint__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `position`
#include "geometry_msgs/msg/point.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"
// Member `velocity`
#include "geometry_msgs/msg/vector3.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  race_msgs__msg__NavigationSetpoint__init(message_memory);
}

void race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_fini_function(void * message_memory)
{
  race_msgs__msg__NavigationSetpoint__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_member_array[7] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__NavigationSetpoint, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__NavigationSetpoint, position),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__NavigationSetpoint, velocity_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__NavigationSetpoint, velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__NavigationSetpoint, yaw),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_rate_valid",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__NavigationSetpoint, yaw_rate_valid),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "yaw_rate",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__NavigationSetpoint, yaw_rate),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_members = {
  "race_msgs__msg",  // message namespace
  "NavigationSetpoint",  // message name
  7,  // number of fields
  sizeof(race_msgs__msg__NavigationSetpoint),
  race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_member_array,  // message members
  race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_init_function,  // function to initialize message memory (memory has to be allocated)
  race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_type_support_handle = {
  0,
  &race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_race_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, race_msgs, msg, NavigationSetpoint)() {
  race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_member_array[1].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Vector3)();
  if (!race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_type_support_handle.typesupport_identifier) {
    race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &race_msgs__msg__NavigationSetpoint__rosidl_typesupport_introspection_c__NavigationSetpoint_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
