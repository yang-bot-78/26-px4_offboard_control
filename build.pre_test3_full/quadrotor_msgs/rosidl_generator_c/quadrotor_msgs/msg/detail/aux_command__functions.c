// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/AuxCommand.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/aux_command__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
quadrotor_msgs__msg__AuxCommand__init(quadrotor_msgs__msg__AuxCommand * msg)
{
  if (!msg) {
    return false;
  }
  // current_yaw
  // kf_correction
  // angle_corrections
  // enable_motors
  // use_external_yaw
  return true;
}

void
quadrotor_msgs__msg__AuxCommand__fini(quadrotor_msgs__msg__AuxCommand * msg)
{
  if (!msg) {
    return;
  }
  // current_yaw
  // kf_correction
  // angle_corrections
  // enable_motors
  // use_external_yaw
}

bool
quadrotor_msgs__msg__AuxCommand__are_equal(const quadrotor_msgs__msg__AuxCommand * lhs, const quadrotor_msgs__msg__AuxCommand * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // current_yaw
  if (lhs->current_yaw != rhs->current_yaw) {
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
  // enable_motors
  if (lhs->enable_motors != rhs->enable_motors) {
    return false;
  }
  // use_external_yaw
  if (lhs->use_external_yaw != rhs->use_external_yaw) {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__AuxCommand__copy(
  const quadrotor_msgs__msg__AuxCommand * input,
  quadrotor_msgs__msg__AuxCommand * output)
{
  if (!input || !output) {
    return false;
  }
  // current_yaw
  output->current_yaw = input->current_yaw;
  // kf_correction
  output->kf_correction = input->kf_correction;
  // angle_corrections
  for (size_t i = 0; i < 2; ++i) {
    output->angle_corrections[i] = input->angle_corrections[i];
  }
  // enable_motors
  output->enable_motors = input->enable_motors;
  // use_external_yaw
  output->use_external_yaw = input->use_external_yaw;
  return true;
}

quadrotor_msgs__msg__AuxCommand *
quadrotor_msgs__msg__AuxCommand__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__AuxCommand * msg = (quadrotor_msgs__msg__AuxCommand *)allocator.allocate(sizeof(quadrotor_msgs__msg__AuxCommand), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__AuxCommand));
  bool success = quadrotor_msgs__msg__AuxCommand__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__AuxCommand__destroy(quadrotor_msgs__msg__AuxCommand * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__AuxCommand__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__AuxCommand__Sequence__init(quadrotor_msgs__msg__AuxCommand__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__AuxCommand * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__AuxCommand *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__AuxCommand), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__AuxCommand__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__AuxCommand__fini(&data[i - 1]);
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
quadrotor_msgs__msg__AuxCommand__Sequence__fini(quadrotor_msgs__msg__AuxCommand__Sequence * array)
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
      quadrotor_msgs__msg__AuxCommand__fini(&array->data[i]);
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

quadrotor_msgs__msg__AuxCommand__Sequence *
quadrotor_msgs__msg__AuxCommand__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__AuxCommand__Sequence * array = (quadrotor_msgs__msg__AuxCommand__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__AuxCommand__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__AuxCommand__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__AuxCommand__Sequence__destroy(quadrotor_msgs__msg__AuxCommand__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__AuxCommand__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__AuxCommand__Sequence__are_equal(const quadrotor_msgs__msg__AuxCommand__Sequence * lhs, const quadrotor_msgs__msg__AuxCommand__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__AuxCommand__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__AuxCommand__Sequence__copy(
  const quadrotor_msgs__msg__AuxCommand__Sequence * input,
  quadrotor_msgs__msg__AuxCommand__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__AuxCommand);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__AuxCommand * data =
      (quadrotor_msgs__msg__AuxCommand *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__AuxCommand__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__AuxCommand__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__AuxCommand__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
