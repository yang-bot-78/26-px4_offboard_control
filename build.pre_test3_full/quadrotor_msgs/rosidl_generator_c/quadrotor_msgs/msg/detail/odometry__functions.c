// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/Odometry.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/odometry__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `curodom`
// Member `kfodom`
#include "nav_msgs/msg/detail/odometry__functions.h"

bool
quadrotor_msgs__msg__Odometry__init(quadrotor_msgs__msg__Odometry * msg)
{
  if (!msg) {
    return false;
  }
  // curodom
  if (!nav_msgs__msg__Odometry__init(&msg->curodom)) {
    quadrotor_msgs__msg__Odometry__fini(msg);
    return false;
  }
  // kfodom
  if (!nav_msgs__msg__Odometry__init(&msg->kfodom)) {
    quadrotor_msgs__msg__Odometry__fini(msg);
    return false;
  }
  // kfid
  // status
  return true;
}

void
quadrotor_msgs__msg__Odometry__fini(quadrotor_msgs__msg__Odometry * msg)
{
  if (!msg) {
    return;
  }
  // curodom
  nav_msgs__msg__Odometry__fini(&msg->curodom);
  // kfodom
  nav_msgs__msg__Odometry__fini(&msg->kfodom);
  // kfid
  // status
}

bool
quadrotor_msgs__msg__Odometry__are_equal(const quadrotor_msgs__msg__Odometry * lhs, const quadrotor_msgs__msg__Odometry * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // curodom
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->curodom), &(rhs->curodom)))
  {
    return false;
  }
  // kfodom
  if (!nav_msgs__msg__Odometry__are_equal(
      &(lhs->kfodom), &(rhs->kfodom)))
  {
    return false;
  }
  // kfid
  if (lhs->kfid != rhs->kfid) {
    return false;
  }
  // status
  if (lhs->status != rhs->status) {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__Odometry__copy(
  const quadrotor_msgs__msg__Odometry * input,
  quadrotor_msgs__msg__Odometry * output)
{
  if (!input || !output) {
    return false;
  }
  // curodom
  if (!nav_msgs__msg__Odometry__copy(
      &(input->curodom), &(output->curodom)))
  {
    return false;
  }
  // kfodom
  if (!nav_msgs__msg__Odometry__copy(
      &(input->kfodom), &(output->kfodom)))
  {
    return false;
  }
  // kfid
  output->kfid = input->kfid;
  // status
  output->status = input->status;
  return true;
}

quadrotor_msgs__msg__Odometry *
quadrotor_msgs__msg__Odometry__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Odometry * msg = (quadrotor_msgs__msg__Odometry *)allocator.allocate(sizeof(quadrotor_msgs__msg__Odometry), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__Odometry));
  bool success = quadrotor_msgs__msg__Odometry__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__Odometry__destroy(quadrotor_msgs__msg__Odometry * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__Odometry__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__Odometry__Sequence__init(quadrotor_msgs__msg__Odometry__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Odometry * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__Odometry *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__Odometry), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__Odometry__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__Odometry__fini(&data[i - 1]);
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
quadrotor_msgs__msg__Odometry__Sequence__fini(quadrotor_msgs__msg__Odometry__Sequence * array)
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
      quadrotor_msgs__msg__Odometry__fini(&array->data[i]);
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

quadrotor_msgs__msg__Odometry__Sequence *
quadrotor_msgs__msg__Odometry__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__Odometry__Sequence * array = (quadrotor_msgs__msg__Odometry__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__Odometry__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__Odometry__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__Odometry__Sequence__destroy(quadrotor_msgs__msg__Odometry__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__Odometry__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__Odometry__Sequence__are_equal(const quadrotor_msgs__msg__Odometry__Sequence * lhs, const quadrotor_msgs__msg__Odometry__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__Odometry__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__Odometry__Sequence__copy(
  const quadrotor_msgs__msg__Odometry__Sequence * input,
  quadrotor_msgs__msg__Odometry__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__Odometry);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__Odometry * data =
      (quadrotor_msgs__msg__Odometry *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__Odometry__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__Odometry__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__Odometry__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
