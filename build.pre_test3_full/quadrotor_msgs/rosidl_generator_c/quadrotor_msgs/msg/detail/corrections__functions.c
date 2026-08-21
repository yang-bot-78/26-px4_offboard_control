// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/Corrections.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/corrections__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
quadrotor_msgs__msg__Corrections__init(quadrotor_msgs__msg__Corrections * msg)
{
  if (!msg) {
    return false;
  }
  // kf_correction
  // angle_corrections
  return true;
}

void
quadrotor_msgs__msg__Corrections__fini(quadrotor_msgs__msg__Corrections * msg)
{
  if (!msg) {
    return;
  }
  // kf_correction
  // angle_corrections
}

bool
quadrotor_msgs__msg__Corrections__are_equal(const quadrotor_msgs__msg__Corrections * lhs, const quadrotor_msgs__msg__Corrections * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // kf_correction
  if (lhs->kf_correction != rhs->kf_correction) {
    return false;
  }
  // angle_corrections
  for (size_t i = 0; i < 2; ++i) {
    if (lhs->angle_corrections[i] != rhs->angle_corrections[i]) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__Corrections__copy(
  const quadrotor_msgs__msg__Corrections * input,
  quadrotor_msgs__msg__Corrections * output)
{
  if (!input || !output) {
    return false;
  }
  // kf_correction
  output->kf_correction = input->kf_correction;
  // angle_corrections
  for (size_t i = 0; i < 2; ++i) {
    output->angle_corrections[i] = input->angle_corrections[i];
  }
  return true;
}

quadrotor_msgs__msg__Corrections *
quadrotor_msgs__msg__Corrections__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Corrections * msg = (quadrotor_msgs__msg__Corrections *)allocator.allocate(sizeof(quadrotor_msgs__msg__Corrections), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__Corrections));
  bool success = quadrotor_msgs__msg__Corrections__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__Corrections__destroy(quadrotor_msgs__msg__Corrections * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__Corrections__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__Corrections__Sequence__init(quadrotor_msgs__msg__Corrections__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Corrections * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__Corrections *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__Corrections), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__Corrections__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__Corrections__fini(&data[i - 1]);
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
quadrotor_msgs__msg__Corrections__Sequence__fini(quadrotor_msgs__msg__Corrections__Sequence * array)
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
      quadrotor_msgs__msg__Corrections__fini(&array->data[i]);
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

quadrotor_msgs__msg__Corrections__Sequence *
quadrotor_msgs__msg__Corrections__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Corrections__Sequence * array = (quadrotor_msgs__msg__Corrections__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__Corrections__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__Corrections__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__Corrections__Sequence__destroy(quadrotor_msgs__msg__Corrections__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__Corrections__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__Corrections__Sequence__are_equal(const quadrotor_msgs__msg__Corrections__Sequence * lhs, const quadrotor_msgs__msg__Corrections__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__Corrections__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__Corrections__Sequence__copy(
  const quadrotor_msgs__msg__Corrections__Sequence * input,
  quadrotor_msgs__msg__Corrections__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__Corrections);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__Corrections * data =
      (quadrotor_msgs__msg__Corrections *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__Corrections__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__Corrections__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__Corrections__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
