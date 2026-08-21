// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from race_msgs:msg/NavigationSetpoint.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__FUNCTIONS_H_
#define RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "race_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "race_msgs/msg/detail/navigation_setpoint__struct.h"

/// Initialize msg/NavigationSetpoint message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * race_msgs__msg__NavigationSetpoint
 * )) before or use
 * race_msgs__msg__NavigationSetpoint__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
bool
race_msgs__msg__NavigationSetpoint__init(race_msgs__msg__NavigationSetpoint * msg);

/// Finalize msg/NavigationSetpoint message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
void
race_msgs__msg__NavigationSetpoint__fini(race_msgs__msg__NavigationSetpoint * msg);

/// Create msg/NavigationSetpoint message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * race_msgs__msg__NavigationSetpoint__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
race_msgs__msg__NavigationSetpoint *
race_msgs__msg__NavigationSetpoint__create();

/// Destroy msg/NavigationSetpoint message.
/**
 * It calls
 * race_msgs__msg__NavigationSetpoint__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
void
race_msgs__msg__NavigationSetpoint__destroy(race_msgs__msg__NavigationSetpoint * msg);

/// Check for msg/NavigationSetpoint message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
bool
race_msgs__msg__NavigationSetpoint__are_equal(const race_msgs__msg__NavigationSetpoint * lhs, const race_msgs__msg__NavigationSetpoint * rhs);

/// Copy a msg/NavigationSetpoint message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
bool
race_msgs__msg__NavigationSetpoint__copy(
  const race_msgs__msg__NavigationSetpoint * input,
  race_msgs__msg__NavigationSetpoint * output);

/// Initialize array of msg/NavigationSetpoint messages.
/**
 * It allocates the memory for the number of elements and calls
 * race_msgs__msg__NavigationSetpoint__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
bool
race_msgs__msg__NavigationSetpoint__Sequence__init(race_msgs__msg__NavigationSetpoint__Sequence * array, size_t size);

/// Finalize array of msg/NavigationSetpoint messages.
/**
 * It calls
 * race_msgs__msg__NavigationSetpoint__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
void
race_msgs__msg__NavigationSetpoint__Sequence__fini(race_msgs__msg__NavigationSetpoint__Sequence * array);

/// Create array of msg/NavigationSetpoint messages.
/**
 * It allocates the memory for the array and calls
 * race_msgs__msg__NavigationSetpoint__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
race_msgs__msg__NavigationSetpoint__Sequence *
race_msgs__msg__NavigationSetpoint__Sequence__create(size_t size);

/// Destroy array of msg/NavigationSetpoint messages.
/**
 * It calls
 * race_msgs__msg__NavigationSetpoint__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
void
race_msgs__msg__NavigationSetpoint__Sequence__destroy(race_msgs__msg__NavigationSetpoint__Sequence * array);

/// Check for msg/NavigationSetpoint message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
bool
race_msgs__msg__NavigationSetpoint__Sequence__are_equal(const race_msgs__msg__NavigationSetpoint__Sequence * lhs, const race_msgs__msg__NavigationSetpoint__Sequence * rhs);

/// Copy an array of msg/NavigationSetpoint messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_race_msgs
bool
race_msgs__msg__NavigationSetpoint__Sequence__copy(
  const race_msgs__msg__NavigationSetpoint__Sequence * input,
  race_msgs__msg__NavigationSetpoint__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // RACE_MSGS__MSG__DETAIL__NAVIGATION_SETPOINT__FUNCTIONS_H_
