// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/aux_command__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "quadrotor_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "quadrotor_msgs/msg/detail/aux_command__struct.h"
#include "quadrotor_msgs/msg/detail/aux_command__functions.h"
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


// forward declare type support functions


using _AuxCommand__ros_msg_type = quadrotor_msgs__msg__AuxCommand;

static bool _AuxCommand__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _AuxCommand__ros_msg_type * ros_message = static_cast<const _AuxCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: current_yaw
  {
    cdr << ros_message->current_yaw;
  }

  // Field name: kf_correction
  {
    cdr << ros_message->kf_correction;
  }

  // Field name: angle_corrections
  {
    size_t size = 2;
    auto array_ptr = ros_message->angle_corrections;
    cdr.serializeArray(array_ptr, size);
  }

  // Field name: enable_motors
  {
    cdr << (ros_message->enable_motors ? true : false);
  }

  // Field name: use_external_yaw
  {
    cdr << (ros_message->use_external_yaw ? true : false);
  }

  return true;
}

static bool _AuxCommand__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _AuxCommand__ros_msg_type * ros_message = static_cast<_AuxCommand__ros_msg_type *>(untyped_ros_message);
  // Field name: current_yaw
  {
    cdr >> ros_message->current_yaw;
  }

  // Field name: kf_correction
  {
    cdr >> ros_message->kf_correction;
  }

  // Field name: angle_corrections
  {
    size_t size = 2;
    auto array_ptr = ros_message->angle_corrections;
    cdr.deserializeArray(array_ptr, size);
  }

  // Field name: enable_motors
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->enable_motors = tmp ? true : false;
  }

  // Field name: use_external_yaw
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->use_external_yaw = tmp ? true : false;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadrotor_msgs
size_t get_serialized_size_quadrotor_msgs__msg__AuxCommand(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _AuxCommand__ros_msg_type * ros_message = static_cast<const _AuxCommand__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name current_yaw
  {
    size_t item_size = sizeof(ros_message->current_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name kf_correction
  {
    size_t item_size = sizeof(ros_message->kf_correction);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angle_corrections
  {
    size_t array_size = 2;
    auto array_ptr = ros_message->angle_corrections;
    (void)array_ptr;
    size_t item_size = sizeof(array_ptr[0]);
    current_alignment += array_size * item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name enable_motors
  {
    size_t item_size = sizeof(ros_message->enable_motors);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name use_external_yaw
  {
    size_t item_size = sizeof(ros_message->use_external_yaw);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _AuxCommand__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_quadrotor_msgs__msg__AuxCommand(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_quadrotor_msgs
size_t max_serialized_size_quadrotor_msgs__msg__AuxCommand(
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

  // member: current_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: kf_correction
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: angle_corrections
  {
    size_t array_size = 2;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: enable_motors
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: use_external_yaw
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = quadrotor_msgs__msg__AuxCommand;
    is_plain =
      (
      offsetof(DataType, use_external_yaw) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _AuxCommand__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_quadrotor_msgs__msg__AuxCommand(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_AuxCommand = {
  "quadrotor_msgs::msg",
  "AuxCommand",
  _AuxCommand__cdr_serialize,
  _AuxCommand__cdr_deserialize,
  _AuxCommand__get_serialized_size,
  _AuxCommand__max_serialized_size
};

static rosidl_message_type_support_t _AuxCommand__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_AuxCommand,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, quadrotor_msgs, msg, AuxCommand)() {
  return &_AuxCommand__type_support;
}

#if defined(__cplusplus)
}
#endif
