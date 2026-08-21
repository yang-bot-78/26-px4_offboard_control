// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice
#include "race_msgs/msg/detail/flight_altitude_reference__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "race_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "race_msgs/msg/detail/flight_altitude_reference__struct.h"
#include "race_msgs/msg/detail/flight_altitude_reference__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_race_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_race_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_race_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _FlightAltitudeReference__ros_msg_type = race_msgs__msg__FlightAltitudeReference;

static bool _FlightAltitudeReference__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _FlightAltitudeReference__ros_msg_type * ros_message = static_cast<const _FlightAltitudeReference__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: flight_id
  {
    cdr << ros_message->flight_id;
  }

  // Field name: valid
  {
    cdr << (ros_message->valid ? true : false);
  }

  // Field name: target_agl_m
  {
    cdr << ros_message->target_agl_m;
  }

  // Field name: min_agl_m
  {
    cdr << ros_message->min_agl_m;
  }

  // Field name: max_agl_m
  {
    cdr << ros_message->max_agl_m;
  }

  // Field name: ground_z_local_ned
  {
    cdr << ros_message->ground_z_local_ned;
  }

  // Field name: target_z_local_ned
  {
    cdr << ros_message->target_z_local_ned;
  }

  // Field name: ground_z_map
  {
    cdr << ros_message->ground_z_map;
  }

  // Field name: target_z_map
  {
    cdr << ros_message->target_z_map;
  }

  return true;
}

static bool _FlightAltitudeReference__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _FlightAltitudeReference__ros_msg_type * ros_message = static_cast<_FlightAltitudeReference__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: flight_id
  {
    cdr >> ros_message->flight_id;
  }

  // Field name: valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->valid = tmp ? true : false;
  }

  // Field name: target_agl_m
  {
    cdr >> ros_message->target_agl_m;
  }

  // Field name: min_agl_m
  {
    cdr >> ros_message->min_agl_m;
  }

  // Field name: max_agl_m
  {
    cdr >> ros_message->max_agl_m;
  }

  // Field name: ground_z_local_ned
  {
    cdr >> ros_message->ground_z_local_ned;
  }

  // Field name: target_z_local_ned
  {
    cdr >> ros_message->target_z_local_ned;
  }

  // Field name: ground_z_map
  {
    cdr >> ros_message->ground_z_map;
  }

  // Field name: target_z_map
  {
    cdr >> ros_message->target_z_map;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_race_msgs
size_t get_serialized_size_race_msgs__msg__FlightAltitudeReference(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _FlightAltitudeReference__ros_msg_type * ros_message = static_cast<const _FlightAltitudeReference__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name flight_id
  {
    size_t item_size = sizeof(ros_message->flight_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name valid
  {
    size_t item_size = sizeof(ros_message->valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_agl_m
  {
    size_t item_size = sizeof(ros_message->target_agl_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name min_agl_m
  {
    size_t item_size = sizeof(ros_message->min_agl_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name max_agl_m
  {
    size_t item_size = sizeof(ros_message->max_agl_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ground_z_local_ned
  {
    size_t item_size = sizeof(ros_message->ground_z_local_ned);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_z_local_ned
  {
    size_t item_size = sizeof(ros_message->target_z_local_ned);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name ground_z_map
  {
    size_t item_size = sizeof(ros_message->ground_z_map);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name target_z_map
  {
    size_t item_size = sizeof(ros_message->target_z_map);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _FlightAltitudeReference__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_race_msgs__msg__FlightAltitudeReference(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_race_msgs
size_t max_serialized_size_race_msgs__msg__FlightAltitudeReference(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: flight_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: target_agl_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: min_agl_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: max_agl_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ground_z_local_ned
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: target_z_local_ned
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: ground_z_map
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: target_z_map
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = race_msgs__msg__FlightAltitudeReference;
    is_plain =
      (
      offsetof(DataType, target_z_map) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _FlightAltitudeReference__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_race_msgs__msg__FlightAltitudeReference(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_FlightAltitudeReference = {
  "race_msgs::msg",
  "FlightAltitudeReference",
  _FlightAltitudeReference__cdr_serialize,
  _FlightAltitudeReference__cdr_deserialize,
  _FlightAltitudeReference__get_serialized_size,
  _FlightAltitudeReference__max_serialized_size
};

static rosidl_message_type_support_t _FlightAltitudeReference__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_FlightAltitudeReference,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, race_msgs, msg, FlightAltitudeReference)() {
  return &_FlightAltitudeReference__type_support;
}

#if defined(__cplusplus)
}
#endif
