// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from fastlio_global_slam:srv/Relocalize.idl
// generated code does not contain a copyright notice

#ifndef FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__FUNCTIONS_H_
#define FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "fastlio_global_slam/msg/rosidl_generator_c__visibility_control.h"

#include "fastlio_global_slam/srv/detail/relocalize__struct.h"

/// Initialize srv/Relocalize message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * fastlio_global_slam__srv__Relocalize_Request
 * )) before or use
 * fastlio_global_slam__srv__Relocalize_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Request__init(fastlio_global_slam__srv__Relocalize_Request * msg);

/// Finalize srv/Relocalize message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Request__fini(fastlio_global_slam__srv__Relocalize_Request * msg);

/// Create srv/Relocalize message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * fastlio_global_slam__srv__Relocalize_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
fastlio_global_slam__srv__Relocalize_Request *
fastlio_global_slam__srv__Relocalize_Request__create();

/// Destroy srv/Relocalize message.
/**
 * It calls
 * fastlio_global_slam__srv__Relocalize_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Request__destroy(fastlio_global_slam__srv__Relocalize_Request * msg);

/// Check for srv/Relocalize message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Request__are_equal(const fastlio_global_slam__srv__Relocalize_Request * lhs, const fastlio_global_slam__srv__Relocalize_Request * rhs);

/// Copy a srv/Relocalize message.
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
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Request__copy(
  const fastlio_global_slam__srv__Relocalize_Request * input,
  fastlio_global_slam__srv__Relocalize_Request * output);

/// Initialize array of srv/Relocalize messages.
/**
 * It allocates the memory for the number of elements and calls
 * fastlio_global_slam__srv__Relocalize_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Request__Sequence__init(fastlio_global_slam__srv__Relocalize_Request__Sequence * array, size_t size);

/// Finalize array of srv/Relocalize messages.
/**
 * It calls
 * fastlio_global_slam__srv__Relocalize_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Request__Sequence__fini(fastlio_global_slam__srv__Relocalize_Request__Sequence * array);

/// Create array of srv/Relocalize messages.
/**
 * It allocates the memory for the array and calls
 * fastlio_global_slam__srv__Relocalize_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
fastlio_global_slam__srv__Relocalize_Request__Sequence *
fastlio_global_slam__srv__Relocalize_Request__Sequence__create(size_t size);

/// Destroy array of srv/Relocalize messages.
/**
 * It calls
 * fastlio_global_slam__srv__Relocalize_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Request__Sequence__destroy(fastlio_global_slam__srv__Relocalize_Request__Sequence * array);

/// Check for srv/Relocalize message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Request__Sequence__are_equal(const fastlio_global_slam__srv__Relocalize_Request__Sequence * lhs, const fastlio_global_slam__srv__Relocalize_Request__Sequence * rhs);

/// Copy an array of srv/Relocalize messages.
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
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Request__Sequence__copy(
  const fastlio_global_slam__srv__Relocalize_Request__Sequence * input,
  fastlio_global_slam__srv__Relocalize_Request__Sequence * output);

/// Initialize srv/Relocalize message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * fastlio_global_slam__srv__Relocalize_Response
 * )) before or use
 * fastlio_global_slam__srv__Relocalize_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Response__init(fastlio_global_slam__srv__Relocalize_Response * msg);

/// Finalize srv/Relocalize message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Response__fini(fastlio_global_slam__srv__Relocalize_Response * msg);

/// Create srv/Relocalize message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * fastlio_global_slam__srv__Relocalize_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
fastlio_global_slam__srv__Relocalize_Response *
fastlio_global_slam__srv__Relocalize_Response__create();

/// Destroy srv/Relocalize message.
/**
 * It calls
 * fastlio_global_slam__srv__Relocalize_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Response__destroy(fastlio_global_slam__srv__Relocalize_Response * msg);

/// Check for srv/Relocalize message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Response__are_equal(const fastlio_global_slam__srv__Relocalize_Response * lhs, const fastlio_global_slam__srv__Relocalize_Response * rhs);

/// Copy a srv/Relocalize message.
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
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Response__copy(
  const fastlio_global_slam__srv__Relocalize_Response * input,
  fastlio_global_slam__srv__Relocalize_Response * output);

/// Initialize array of srv/Relocalize messages.
/**
 * It allocates the memory for the number of elements and calls
 * fastlio_global_slam__srv__Relocalize_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Response__Sequence__init(fastlio_global_slam__srv__Relocalize_Response__Sequence * array, size_t size);

/// Finalize array of srv/Relocalize messages.
/**
 * It calls
 * fastlio_global_slam__srv__Relocalize_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Response__Sequence__fini(fastlio_global_slam__srv__Relocalize_Response__Sequence * array);

/// Create array of srv/Relocalize messages.
/**
 * It allocates the memory for the array and calls
 * fastlio_global_slam__srv__Relocalize_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
fastlio_global_slam__srv__Relocalize_Response__Sequence *
fastlio_global_slam__srv__Relocalize_Response__Sequence__create(size_t size);

/// Destroy array of srv/Relocalize messages.
/**
 * It calls
 * fastlio_global_slam__srv__Relocalize_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
void
fastlio_global_slam__srv__Relocalize_Response__Sequence__destroy(fastlio_global_slam__srv__Relocalize_Response__Sequence * array);

/// Check for srv/Relocalize message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Response__Sequence__are_equal(const fastlio_global_slam__srv__Relocalize_Response__Sequence * lhs, const fastlio_global_slam__srv__Relocalize_Response__Sequence * rhs);

/// Copy an array of srv/Relocalize messages.
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
ROSIDL_GENERATOR_C_PUBLIC_fastlio_global_slam
bool
fastlio_global_slam__srv__Relocalize_Response__Sequence__copy(
  const fastlio_global_slam__srv__Relocalize_Response__Sequence * input,
  fastlio_global_slam__srv__Relocalize_Response__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // FASTLIO_GLOBAL_SLAM__SRV__DETAIL__RELOCALIZE__FUNCTIONS_H_
