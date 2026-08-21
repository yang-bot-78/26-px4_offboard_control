// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/StatusData.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/status_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
quadrotor_msgs__msg__StatusData__init(quadrotor_msgs__msg__StatusData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__StatusData__fini(msg);
    return false;
  }
  // loop_rate
  // voltage
  // seq
  return true;
}

void
quadrotor_msgs__msg__StatusData__fini(quadrotor_msgs__msg__StatusData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // loop_rate
  // voltage
  // seq
}

bool
quadrotor_msgs__msg__StatusData__are_equal(const quadrotor_msgs__msg__StatusData * lhs, const quadrotor_msgs__msg__StatusData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // loop_rate
  if (lhs->loop_rate != rhs->loop_rate) {
    return false;
  }
  // voltage
  if (lhs->voltage != rhs->voltage) {
    return false;
  }
  // seq
  if (lhs->seq != rhs->seq) {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__StatusData__copy(
  const quadrotor_msgs__msg__StatusData * input,
  quadrotor_msgs__msg__StatusData * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // loop_rate
  output->loop_rate = input->loop_rate;
  // voltage
  output->voltage = input->voltage;
  // seq
  output->seq = input->seq;
  return true;
}

quadrotor_msgs__msg__StatusData *
quadrotor_msgs__msg__StatusData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__StatusData * msg = (quadrotor_msgs__msg__StatusData *)allocator.allocate(sizeof(quadrotor_msgs__msg__StatusData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__StatusData));
  bool success = quadrotor_msgs__msg__StatusData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__StatusData__destroy(quadrotor_msgs__msg__StatusData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__StatusData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__StatusData__Sequence__init(quadrotor_msgs__msg__StatusData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__StatusData * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__StatusData *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__StatusData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__StatusData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__StatusData__fini(&data[i - 1]);
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
quadrotor_msgs__msg__StatusData__Sequence__fini(quadrotor_msgs__msg__StatusData__Sequence * array)
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
      quadrotor_msgs__msg__StatusData__fini(&array->data[i]);
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

quadrotor_msgs__msg__StatusData__Sequence *
quadrotor_msgs__msg__StatusData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__StatusData__Sequence * array = (quadrotor_msgs__msg__StatusData__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__StatusData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__StatusData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__StatusData__Sequence__destroy(quadrotor_msgs__msg__StatusData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__StatusData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__StatusData__Sequence__are_equal(const quadrotor_msgs__msg__StatusData__Sequence * lhs, const quadrotor_msgs__msg__StatusData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__StatusData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__StatusData__Sequence__copy(
  const quadrotor_msgs__msg__StatusData__Sequence * input,
  quadrotor_msgs__msg__StatusData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__StatusData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__StatusData * data =
      (quadrotor_msgs__msg__StatusData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__StatusData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__StatusData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__StatusData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
