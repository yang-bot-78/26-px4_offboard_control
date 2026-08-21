// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/LQRTrajectory.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/lqr_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `debug_info`
#include "rosidl_runtime_c/string_functions.h"

bool
quadrotor_msgs__msg__LQRTrajectory__init(quadrotor_msgs__msg__LQRTrajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__LQRTrajectory__fini(msg);
    return false;
  }
  // trajectory_id
  // action
  // r
  // start_yaw
  // final_yaw
  // s0
  // ut
  // sf
  // t_f
  // debug_info
  if (!rosidl_runtime_c__String__init(&msg->debug_info)) {
    quadrotor_msgs__msg__LQRTrajectory__fini(msg);
    return false;
  }
  return true;
}

void
quadrotor_msgs__msg__LQRTrajectory__fini(quadrotor_msgs__msg__LQRTrajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // trajectory_id
  // action
  // r
  // start_yaw
  // final_yaw
  // s0
  // ut
  // sf
  // t_f
  // debug_info
  rosidl_runtime_c__String__fini(&msg->debug_info);
}

bool
quadrotor_msgs__msg__LQRTrajectory__are_equal(const quadrotor_msgs__msg__LQRTrajectory * lhs, const quadrotor_msgs__msg__LQRTrajectory * rhs)
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
  // trajectory_id
  if (lhs->trajectory_id != rhs->trajectory_id) {
    return false;
  }
  // action
  if (lhs->action != rhs->action) {
    return false;
  }
  // r
  if (lhs->r != rhs->r) {
    return false;
  }
  // start_yaw
  if (lhs->start_yaw != rhs->start_yaw) {
    return false;
  }
  // final_yaw
  if (lhs->final_yaw != rhs->final_yaw) {
    return false;
  }
  // s0
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->s0[i] != rhs->s0[i]) {
      return false;
    }
  }
  // ut
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->ut[i] != rhs->ut[i]) {
      return false;
    }
  }
  // sf
  for (size_t i = 0; i < 6; ++i) {
    if (lhs->sf[i] != rhs->sf[i]) {
      return false;
    }
  }
  // t_f
  if (lhs->t_f != rhs->t_f) {
    return false;
  }
  // debug_info
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->debug_info), &(rhs->debug_info)))
  {
    return false;
  }
  return true;
}

bool
quadrotor_msgs__msg__LQRTrajectory__copy(
  const quadrotor_msgs__msg__LQRTrajectory * input,
  quadrotor_msgs__msg__LQRTrajectory * output)
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
  // trajectory_id
  output->trajectory_id = input->trajectory_id;
  // action
  output->action = input->action;
  // r
  output->r = input->r;
  // start_yaw
  output->start_yaw = input->start_yaw;
  // final_yaw
  output->final_yaw = input->final_yaw;
  // s0
  for (size_t i = 0; i < 6; ++i) {
    output->s0[i] = input->s0[i];
  }
  // ut
  for (size_t i = 0; i < 3; ++i) {
    output->ut[i] = input->ut[i];
  }
  // sf
  for (size_t i = 0; i < 6; ++i) {
    output->sf[i] = input->sf[i];
  }
  // t_f
  output->t_f = input->t_f;
  // debug_info
  if (!rosidl_runtime_c__String__copy(
      &(input->debug_info), &(output->debug_info)))
  {
    return false;
  }
  return true;
}

quadrotor_msgs__msg__LQRTrajectory *
quadrotor_msgs__msg__LQRTrajectory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__LQRTrajectory * msg = (quadrotor_msgs__msg__LQRTrajectory *)allocator.allocate(sizeof(quadrotor_msgs__msg__LQRTrajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__LQRTrajectory));
  bool success = quadrotor_msgs__msg__LQRTrajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__LQRTrajectory__destroy(quadrotor_msgs__msg__LQRTrajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__LQRTrajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__LQRTrajectory__Sequence__init(quadrotor_msgs__msg__LQRTrajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__LQRTrajectory * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__LQRTrajectory *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__LQRTrajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__LQRTrajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__LQRTrajectory__fini(&data[i - 1]);
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
quadrotor_msgs__msg__LQRTrajectory__Sequence__fini(quadrotor_msgs__msg__LQRTrajectory__Sequence * array)
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
      quadrotor_msgs__msg__LQRTrajectory__fini(&array->data[i]);
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

quadrotor_msgs__msg__LQRTrajectory__Sequence *
quadrotor_msgs__msg__LQRTrajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__LQRTrajectory__Sequence * array = (quadrotor_msgs__msg__LQRTrajectory__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__LQRTrajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__LQRTrajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__LQRTrajectory__Sequence__destroy(quadrotor_msgs__msg__LQRTrajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__LQRTrajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__LQRTrajectory__Sequence__are_equal(const quadrotor_msgs__msg__LQRTrajectory__Sequence * lhs, const quadrotor_msgs__msg__LQRTrajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__LQRTrajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__LQRTrajectory__Sequence__copy(
  const quadrotor_msgs__msg__LQRTrajectory__Sequence * input,
  quadrotor_msgs__msg__LQRTrajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__LQRTrajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__LQRTrajectory * data =
      (quadrotor_msgs__msg__LQRTrajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__LQRTrajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__LQRTrajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__LQRTrajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
