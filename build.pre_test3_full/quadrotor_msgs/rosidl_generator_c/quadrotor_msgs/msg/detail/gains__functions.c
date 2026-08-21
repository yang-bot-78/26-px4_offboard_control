// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/Gains.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/gains__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
quadrotor_msgs__msg__Gains__init(quadrotor_msgs__msg__Gains * msg)
{
  if (!msg) {
    return false;
  }
  // kp
  // kd
  // kp_yaw
  // kd_yaw
  return true;
}

void
quadrotor_msgs__msg__Gains__fini(quadrotor_msgs__msg__Gains * msg)
{
  if (!msg) {
    return;
  }
  // kp
  // kd
  // kp_yaw
  // kd_yaw
}

bool
quadrotor_msgs__msg__Gains__are_equal(const quadrotor_msgs__msg__Gains * lhs, const quadrotor_msgs__msg__Gains * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // kp
  if (lhs->kp != rhs->kp) {
    return false;
  }
  // kd
  if (lhs->kd != rhs->kd) {
    return false;
  }
  // kp_yaw
  if (lhs->kp_yaw != rhs->kp_yaw) {
    return false;
  }
  // kd_yaw
  if (lhs->kd_yaw != rhs->kd_yaw) {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__Gains__copy(
  const quadrotor_msgs__msg__Gains * input,
  quadrotor_msgs__msg__Gains * output)
{
  if (!input || !output) {
    return false;
  }
  // kp
  output->kp = input->kp;
  // kd
  output->kd = input->kd;
  // kp_yaw
  output->kp_yaw = input->kp_yaw;
  // kd_yaw
  output->kd_yaw = input->kd_yaw;
  return true;
}

quadrotor_msgs__msg__Gains *
quadrotor_msgs__msg__Gains__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Gains * msg = (quadrotor_msgs__msg__Gains *)allocator.allocate(sizeof(quadrotor_msgs__msg__Gains), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__Gains));
  bool success = quadrotor_msgs__msg__Gains__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__Gains__destroy(quadrotor_msgs__msg__Gains * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__Gains__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__Gains__Sequence__init(quadrotor_msgs__msg__Gains__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Gains * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__Gains *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__Gains), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__Gains__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__Gains__fini(&data[i - 1]);
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
quadrotor_msgs__msg__Gains__Sequence__fini(quadrotor_msgs__msg__Gains__Sequence * array)
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
      quadrotor_msgs__msg__Gains__fini(&array->data[i]);
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

quadrotor_msgs__msg__Gains__Sequence *
quadrotor_msgs__msg__Gains__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Gains__Sequence * array = (quadrotor_msgs__msg__Gains__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__Gains__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__Gains__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__Gains__Sequence__destroy(quadrotor_msgs__msg__Gains__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__Gains__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__Gains__Sequence__are_equal(const quadrotor_msgs__msg__Gains__Sequence * lhs, const quadrotor_msgs__msg__Gains__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__Gains__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__Gains__Sequence__copy(
  const quadrotor_msgs__msg__Gains__Sequence * input,
  quadrotor_msgs__msg__Gains__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__Gains);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__Gains * data =
      (quadrotor_msgs__msg__Gains *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__Gains__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__Gains__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__Gains__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
