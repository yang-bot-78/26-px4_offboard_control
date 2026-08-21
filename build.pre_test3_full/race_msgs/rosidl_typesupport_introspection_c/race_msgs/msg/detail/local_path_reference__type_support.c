// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from race_msgs:msg/LocalPathReference.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "race_msgs/msg/detail/local_path_reference__rosidl_typesupport_introspection_c.h"
#include "race_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "race_msgs/msg/detail/local_path_reference__functions.h"
#include "race_msgs/msg/detail/local_path_reference__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `local_goal`
// Member `points`
// Member `continuation_points`
#include "geometry_msgs/msg/point.h"
// Member `local_goal`
// Member `points`
// Member `continuation_points`
#include "geometry_msgs/msg/detail/point__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  race_msgs__msg__LocalPathReference__init(message_memory);
}

void race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_fini_function(void * message_memory)
{
  race_msgs__msg__LocalPathReference__fini(message_memory);
}

size_t race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__size_function__LocalPathReference__points(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_const_function__LocalPathReference__points(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_function__LocalPathReference__points(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__fetch_function__LocalPathReference__points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_const_function__LocalPathReference__points(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__assign_function__LocalPathReference__points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_function__LocalPathReference__points(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__resize_function__LocalPathReference__points(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

size_t race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__size_function__LocalPathReference__continuation_points(
  const void * untyped_member)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return member->size;
}

const void * race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_const_function__LocalPathReference__continuation_points(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Point__Sequence * member =
    (const geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void * race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_function__LocalPathReference__continuation_points(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  return &member->data[index];
}

void race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__fetch_function__LocalPathReference__continuation_points(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Point * item =
    ((const geometry_msgs__msg__Point *)
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_const_function__LocalPathReference__continuation_points(untyped_member, index));
  geometry_msgs__msg__Point * value =
    (geometry_msgs__msg__Point *)(untyped_value);
  *value = *item;
}

void race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__assign_function__LocalPathReference__continuation_points(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Point * item =
    ((geometry_msgs__msg__Point *)
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_function__LocalPathReference__continuation_points(untyped_member, index));
  const geometry_msgs__msg__Point * value =
    (const geometry_msgs__msg__Point *)(untyped_value);
  *item = *value;
}

bool race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__resize_function__LocalPathReference__continuation_points(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Point__Sequence * member =
    (geometry_msgs__msg__Point__Sequence *)(untyped_member);
  geometry_msgs__msg__Point__Sequence__fini(member);
  return geometry_msgs__msg__Point__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_member_array[8] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "global_path_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, global_path_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_goal_seq",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, local_goal_seq),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "local_goal",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, local_goal),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, points),  // bytes offset in struct
    NULL,  // default value
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__size_function__LocalPathReference__points,  // size() function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_const_function__LocalPathReference__points,  // get_const(index) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_function__LocalPathReference__points,  // get(index) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__fetch_function__LocalPathReference__points,  // fetch(index, &value) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__assign_function__LocalPathReference__points,  // assign(index, value) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__resize_function__LocalPathReference__points  // resize(index) function pointer
  },
  {
    "continuation_points",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, continuation_points),  // bytes offset in struct
    NULL,  // default value
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__size_function__LocalPathReference__continuation_points,  // size() function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_const_function__LocalPathReference__continuation_points,  // get_const(index) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__get_function__LocalPathReference__continuation_points,  // get(index) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__fetch_function__LocalPathReference__continuation_points,  // fetch(index, &value) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__assign_function__LocalPathReference__continuation_points,  // assign(index, value) function pointer
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__resize_function__LocalPathReference__continuation_points  // resize(index) function pointer
  },
  {
    "arc_length",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, arc_length),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "minimum_clearance",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(race_msgs__msg__LocalPathReference, minimum_clearance),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_members = {
  "race_msgs__msg",  // message namespace
  "LocalPathReference",  // message name
  8,  // number of fields
  sizeof(race_msgs__msg__LocalPathReference),
  race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_member_array,  // message members
  race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_init_function,  // function to initialize message memory (memory has to be allocated)
  race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_type_support_handle = {
  0,
  &race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_race_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, race_msgs, msg, LocalPathReference)() {
  race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_member_array[3].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_member_array[4].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_member_array[5].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Point)();
  if (!race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_type_support_handle.typesupport_identifier) {
    race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &race_msgs__msg__LocalPathReference__rosidl_typesupport_introspection_c__LocalPathReference_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
