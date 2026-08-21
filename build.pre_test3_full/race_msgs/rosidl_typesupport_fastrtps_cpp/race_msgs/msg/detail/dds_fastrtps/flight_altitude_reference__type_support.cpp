// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice
#include "race_msgs/msg/detail/flight_altitude_reference__rosidl_typesupport_fastrtps_cpp.hpp"
#include "race_msgs/msg/detail/flight_altitude_reference__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace race_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_race_msgs
cdr_serialize(
  const race_msgs::msg::FlightAltitudeReference & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: flight_id
  cdr << ros_message.flight_id;
  // Member: valid
  cdr << (ros_message.valid ? true : false);
  // Member: target_agl_m
  cdr << ros_message.target_agl_m;
  // Member: min_agl_m
  cdr << ros_message.min_agl_m;
  // Member: max_agl_m
  cdr << ros_message.max_agl_m;
  // Member: ground_z_local_ned
  cdr << ros_message.ground_z_local_ned;
  // Member: target_z_local_ned
  cdr << ros_message.target_z_local_ned;
  // Member: ground_z_map
  cdr << ros_message.ground_z_map;
  // Member: target_z_map
  cdr << ros_message.target_z_map;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_race_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  race_msgs::msg::FlightAltitudeReference & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: flight_id
  cdr >> ros_message.flight_id;

  // Member: valid
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message.valid = tmp ? true : false;
  }

  // Member: target_agl_m
  cdr >> ros_message.target_agl_m;

  // Member: min_agl_m
  cdr >> ros_message.min_agl_m;

  // Member: max_agl_m
  cdr >> ros_message.max_agl_m;

  // Member: ground_z_local_ned
  cdr >> ros_message.ground_z_local_ned;

  // Member: target_z_local_ned
  cdr >> ros_message.target_z_local_ned;

  // Member: ground_z_map
  cdr >> ros_message.ground_z_map;

  // Member: target_z_map
  cdr >> ros_message.target_z_map;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_race_msgs
get_serialized_size(
  const race_msgs::msg::FlightAltitudeReference & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: flight_id
  {
    size_t item_size = sizeof(ros_message.flight_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: valid
  {
    size_t item_size = sizeof(ros_message.valid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_agl_m
  {
    size_t item_size = sizeof(ros_message.target_agl_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: min_agl_m
  {
    size_t item_size = sizeof(ros_message.min_agl_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: max_agl_m
  {
    size_t item_size = sizeof(ros_message.max_agl_m);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ground_z_local_ned
  {
    size_t item_size = sizeof(ros_message.ground_z_local_ned);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_z_local_ned
  {
    size_t item_size = sizeof(ros_message.target_z_local_ned);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: ground_z_map
  {
    size_t item_size = sizeof(ros_message.ground_z_map);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: target_z_map
  {
    size_t item_size = sizeof(ros_message.target_z_map);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_race_msgs
max_serialized_size_FlightAltitudeReference(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: flight_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: valid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  // Member: target_agl_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: min_agl_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: max_agl_m
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ground_z_local_ned
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: target_z_local_ned
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: ground_z_map
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }

  // Member: target_z_map
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
    using DataType = race_msgs::msg::FlightAltitudeReference;
    is_plain =
      (
      offsetof(DataType, target_z_map) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _FlightAltitudeReference__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const race_msgs::msg::FlightAltitudeReference *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _FlightAltitudeReference__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<race_msgs::msg::FlightAltitudeReference *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _FlightAltitudeReference__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const race_msgs::msg::FlightAltitudeReference *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _FlightAltitudeReference__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_FlightAltitudeReference(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _FlightAltitudeReference__callbacks = {
  "race_msgs::msg",
  "FlightAltitudeReference",
  _FlightAltitudeReference__cdr_serialize,
  _FlightAltitudeReference__cdr_deserialize,
  _FlightAltitudeReference__get_serialized_size,
  _FlightAltitudeReference__max_serialized_size
};

static rosidl_message_type_support_t _FlightAltitudeReference__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_FlightAltitudeReference__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace race_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_race_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<race_msgs::msg::FlightAltitudeReference>()
{
  return &race_msgs::msg::typesupport_fastrtps_cpp::_FlightAltitudeReference__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, race_msgs, msg, FlightAltitudeReference)() {
  return &race_msgs::msg::typesupport_fastrtps_cpp::_FlightAltitudeReference__handle;
}

#ifdef __cplusplus
}
#endif
