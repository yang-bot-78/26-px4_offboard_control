// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/TRPYCommand.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/trpy_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `aux`
#include "quadrotor_msgs/msg/detail/aux_command__functions.h"

bool
quadrotor_msgs__msg__TRPYCommand__init(quadrotor_msgs__msg__TRPYCommand * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__TRPYCommand__fini(msg);
    return false;
  }
  // thrust
  // roll
  // pitch
  // yaw
  // aux
  if (!quadrotor_msgs__msg__AuxCommand__init(&msg->aux)) {
    quadrotor_msgs__msg__TRPYCommand__fini(msg);
    return false;
  }
  return true;
}

void
quadrotor_msgs__msg__TRPYCommand__fini(quadrotor_msgs__msg__TRPYCommand * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // thrust
  // roll
  // pitch
  // yaw
  // aux
  quadrotor_msgs__msg__AuxCommand__fini(&msg->aux);
}

bool
quadrotor_msgs__msg__TRPYCommand__are_equal(const quadrotor_msgs__msg__TRPYCommand * lhs, const quadrotor_msgs__msg__TRPYCommand * rhs)
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
  // thrust
  if (lhs->thrust != rhs->thrust) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // aux
  if (!quadrotor_msgs__msg__AuxCommand__are_equal(
      &(lhs->aux), &(rhs->aux)))
  {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__TRPYCommand__copy(
  const quadrotor_msgs__msg__TRPYCommand * input,
  quadrotor_msgs__msg__TRPYCommand * output)
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
  // thrust
  output->thrust = input->thrust;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  // aux
  if (!quadrotor_msgs__msg__AuxCommand__copy(
      &(input->aux), &(output->aux)))
  {
    return false;
  }
  return true;
}

quadrotor_msgs__msg__TRPYCommand *
quadrotor_msgs__msg__TRPYCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__TRPYCommand * msg = (quadrotor_msgs__msg__TRPYCommand *)allocator.allocate(sizeof(quadrotor_msgs__msg__TRPYCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__TRPYCommand));
  bool success = quadrotor_msgs__msg__TRPYCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__TRPYCommand__destroy(quadrotor_msgs__msg__TRPYCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__TRPYCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__TRPYCommand__Sequence__init(quadrotor_msgs__msg__TRPYCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__TRPYCommand * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__TRPYCommand *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__TRPYCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__TRPYCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__TRPYCommand__fini(&data[i - 1]);
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
quadrotor_msgs__msg__TRPYCommand__Sequence__fini(quadrotor_msgs__msg__TRPYCommand__Sequence * array)
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
      quadrotor_msgs__msg__TRPYCommand__fini(&array->data[i]);
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

quadrotor_msgs__msg__TRPYCommand__Sequence *
quadrotor_msgs__msg__TRPYCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__TRPYCommand__Sequence * array = (quadrotor_msgs__msg__TRPYCommand__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__TRPYCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__TRPYCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__TRPYCommand__Sequence__destroy(quadrotor_msgs__msg__TRPYCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__TRPYCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__TRPYCommand__Sequence__are_equal(const quadrotor_msgs__msg__TRPYCommand__Sequence * lhs, const quadrotor_msgs__msg__TRPYCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__TRPYCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__TRPYCommand__Sequence__copy(
  const quadrotor_msgs__msg__TRPYCommand__Sequence * input,
  quadrotor_msgs__msg__TRPYCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__TRPYCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__TRPYCommand * data =
      (quadrotor_msgs__msg__TRPYCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__TRPYCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__TRPYCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__TRPYCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
