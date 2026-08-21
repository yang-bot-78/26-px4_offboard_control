// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from fastlio_global_slam:srv/SaveMap.idl
// generated code does not contain a copyright notice
#include "fastlio_global_slam/srv/detail/save_map__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `directory`
#include "rosidl_runtime_c/string_functions.h"

bool
fastlio_global_slam__srv__SaveMap_Request__init(fastlio_global_slam__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return false;
  }
  // directory
  if (!rosidl_runtime_c__String__init(&msg->directory)) {
    fastlio_global_slam__srv__SaveMap_Request__fini(msg);
    return false;
  }
  // resolution
  return true;
}

void
fastlio_global_slam__srv__SaveMap_Request__fini(fastlio_global_slam__srv__SaveMap_Request * msg)
{
  if (!msg) {
    return;
  }
  // directory
  rosidl_runtime_c__String__fini(&msg->directory);
  // resolution
}

bool
fastlio_global_slam__srv__SaveMap_Request__are_equal(const fastlio_global_slam__srv__SaveMap_Request * lhs, const fastlio_global_slam__srv__SaveMap_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // directory
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->directory), &(rhs->directory)))
  {
    return false;
  }
  // resolution
  if (lhs->resolution != rhs->resolution) {
    return false;
  }
  return true;
}

bool
fastlio_global_slam__srv__SaveMap_Request__copy(
  const fastlio_global_slam__srv__SaveMap_Request * input,
  fastlio_global_slam__srv__SaveMap_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // directory
  if (!rosidl_runtime_c__String__copy(
      &(input->directory), &(output->directory)))
  {
    return false;
  }
  // resolution
  output->resolution = input->resolution;
  return true;
}

fastlio_global_slam__srv__SaveMap_Request *
fastlio_global_slam__srv__SaveMap_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fastlio_global_slam__srv__SaveMap_Request * msg = (fastlio_global_slam__srv__SaveMap_Request *)allocator.allocate(sizeof(fastlio_global_slam__srv__SaveMap_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(fastlio_global_slam__srv__SaveMap_Request));
  bool success = fastlio_global_slam__srv__SaveMap_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
fastlio_global_slam__srv__SaveMap_Request__destroy(fastlio_global_slam__srv__SaveMap_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    fastlio_global_slam__srv__SaveMap_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
fastlio_global_slam__srv__SaveMap_Request__Sequence__init(fastlio_global_slam__srv__SaveMap_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fastlio_global_slam__srv__SaveMap_Request * data = NULL;

  if (size) {
    data = (fastlio_global_slam__srv__SaveMap_Request *)allocator.zero_allocate(size, sizeof(fastlio_global_slam__srv__SaveMap_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = fastlio_global_slam__srv__SaveMap_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        fastlio_global_slam__srv__SaveMap_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
fastlio_global_slam__srv__SaveMap_Request__Sequence__fini(fastlio_global_slam__srv__SaveMap_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      fastlio_global_slam__srv__SaveMap_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

fastlio_global_slam__srv__SaveMap_Request__Sequence *
fastlio_global_slam__srv__SaveMap_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fastlio_global_slam__srv__SaveMap_Request__Sequence * array = (fastlio_global_slam__srv__SaveMap_Request__Sequence *)allocator.allocate(sizeof(fastlio_global_slam__srv__SaveMap_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = fastlio_global_slam__srv__SaveMap_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
fastlio_global_slam__srv__SaveMap_Request__Sequence__destroy(fastlio_global_slam__srv__SaveMap_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    fastlio_global_slam__srv__SaveMap_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
fastlio_global_slam__srv__SaveMap_Request__Sequence__are_equal(const fastlio_global_slam__srv__SaveMap_Request__Sequence * lhs, const fastlio_global_slam__srv__SaveMap_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!fastlio_global_slam__srv__SaveMap_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
fastlio_global_slam__srv__SaveMap_Request__Sequence__copy(
  const fastlio_global_slam__srv__SaveMap_Request__Sequence * input,
  fastlio_global_slam__srv__SaveMap_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(fastlio_global_slam__srv__SaveMap_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    fastlio_global_slam__srv__SaveMap_Request * data =
      (fastlio_global_slam__srv__SaveMap_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!fastlio_global_slam__srv__SaveMap_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          fastlio_global_slam__srv__SaveMap_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!fastlio_global_slam__srv__SaveMap_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `message`
// already included above
// #include "rosidl_runtime_c/string_functions.h"

bool
fastlio_global_slam__srv__SaveMap_Response__init(fastlio_global_slam__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return false;
  }
  // success
  // message
  if (!rosidl_runtime_c__String__init(&msg->message)) {
    fastlio_global_slam__srv__SaveMap_Response__fini(msg);
    return false;
  }
  return true;
}

void
fastlio_global_slam__srv__SaveMap_Response__fini(fastlio_global_slam__srv__SaveMap_Response * msg)
{
  if (!msg) {
    return;
  }
  // success
  // message
  rosidl_runtime_c__String__fini(&msg->message);
}

bool
fastlio_global_slam__srv__SaveMap_Response__are_equal(const fastlio_global_slam__srv__SaveMap_Response * lhs, const fastlio_global_slam__srv__SaveMap_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // success
  if (lhs->success != rhs->success) {
    return false;
  }
  // message
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->message), &(rhs->message)))
  {
    return false;
  }
  return true;
}

bool
fastlio_global_slam__srv__SaveMap_Response__copy(
  const fastlio_global_slam__srv__SaveMap_Response * input,
  fastlio_global_slam__srv__SaveMap_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // success
  output->success = input->success;
  // message
  if (!rosidl_runtime_c__String__copy(
      &(input->message), &(output->message)))
  {
    return false;
  }
  return true;
}

fastlio_global_slam__srv__SaveMap_Response *
fastlio_global_slam__srv__SaveMap_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fastlio_global_slam__srv__SaveMap_Response * msg = (fastlio_global_slam__srv__SaveMap_Response *)allocator.allocate(sizeof(fastlio_global_slam__srv__SaveMap_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(fastlio_global_slam__srv__SaveMap_Response));
  bool success = fastlio_global_slam__srv__SaveMap_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
fastlio_global_slam__srv__SaveMap_Response__destroy(fastlio_global_slam__srv__SaveMap_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    fastlio_global_slam__srv__SaveMap_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
fastlio_global_slam__srv__SaveMap_Response__Sequence__init(fastlio_global_slam__srv__SaveMap_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fastlio_global_slam__srv__SaveMap_Response * data = NULL;

  if (size) {
    data = (fastlio_global_slam__srv__SaveMap_Response *)allocator.zero_allocate(size, sizeof(fastlio_global_slam__srv__SaveMap_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = fastlio_global_slam__srv__SaveMap_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        fastlio_global_slam__srv__SaveMap_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
fastlio_global_slam__srv__SaveMap_Response__Sequence__fini(fastlio_global_slam__srv__SaveMap_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      fastlio_global_slam__srv__SaveMap_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

fastlio_global_slam__srv__SaveMap_Response__Sequence *
fastlio_global_slam__srv__SaveMap_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  fastlio_global_slam__srv__SaveMap_Response__Sequence * array = (fastlio_global_slam__srv__SaveMap_Response__Sequence *)allocator.allocate(sizeof(fastlio_global_slam__srv__SaveMap_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = fastlio_global_slam__srv__SaveMap_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
fastlio_global_slam__srv__SaveMap_Response__Sequence__destroy(fastlio_global_slam__srv__SaveMap_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    fastlio_global_slam__srv__SaveMap_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
fastlio_global_slam__srv__SaveMap_Response__Sequence__are_equal(const fastlio_global_slam__srv__SaveMap_Response__Sequence * lhs, const fastlio_global_slam__srv__SaveMap_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!fastlio_global_slam__srv__SaveMap_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
fastlio_global_slam__srv__SaveMap_Response__Sequence__copy(
  const fastlio_global_slam__srv__SaveMap_Response__Sequence * input,
  fastlio_global_slam__srv__SaveMap_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(fastlio_global_slam__srv__SaveMap_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    fastlio_global_slam__srv__SaveMap_Response * data =
      (fastlio_global_slam__srv__SaveMap_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!fastlio_global_slam__srv__SaveMap_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          fastlio_global_slam__srv__SaveMap_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!fastlio_global_slam__srv__SaveMap_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
