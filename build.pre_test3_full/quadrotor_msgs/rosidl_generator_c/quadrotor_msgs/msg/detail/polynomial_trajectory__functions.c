// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/PolynomialTrajectory.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/polynomial_trajectory__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `coef_x`
// Member `coef_y`
// Member `coef_z`
// Member `time`
// Member `order`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `debug_info`
#include "rosidl_runtime_c/string_functions.h"

bool
quadrotor_msgs__msg__PolynomialTrajectory__init(quadrotor_msgs__msg__PolynomialTrajectory * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
    return false;
  }
  // trajectory_id
  // action
  // num_order
  // num_segment
  // start_yaw
  // final_yaw
  // coef_x
  if (!rosidl_runtime_c__double__Sequence__init(&msg->coef_x, 0)) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
    return false;
  }
  // coef_y
  if (!rosidl_runtime_c__double__Sequence__init(&msg->coef_y, 0)) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
    return false;
  }
  // coef_z
  if (!rosidl_runtime_c__double__Sequence__init(&msg->coef_z, 0)) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
    return false;
  }
  // time
  if (!rosidl_runtime_c__double__Sequence__init(&msg->time, 0)) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
    return false;
  }
  // mag_coeff
  // order
  if (!rosidl_runtime_c__uint32__Sequence__init(&msg->order, 0)) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
    return false;
  }
  // debug_info
  if (!rosidl_runtime_c__String__init(&msg->debug_info)) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
    return false;
  }
  return true;
}

void
quadrotor_msgs__msg__PolynomialTrajectory__fini(quadrotor_msgs__msg__PolynomialTrajectory * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // trajectory_id
  // action
  // num_order
  // num_segment
  // start_yaw
  // final_yaw
  // coef_x
  rosidl_runtime_c__double__Sequence__fini(&msg->coef_x);
  // coef_y
  rosidl_runtime_c__double__Sequence__fini(&msg->coef_y);
  // coef_z
  rosidl_runtime_c__double__Sequence__fini(&msg->coef_z);
  // time
  rosidl_runtime_c__double__Sequence__fini(&msg->time);
  // mag_coeff
  // order
  rosidl_runtime_c__uint32__Sequence__fini(&msg->order);
  // debug_info
  rosidl_runtime_c__String__fini(&msg->debug_info);
}

bool
quadrotor_msgs__msg__PolynomialTrajectory__are_equal(const quadrotor_msgs__msg__PolynomialTrajectory * lhs, const quadrotor_msgs__msg__PolynomialTrajectory * rhs)
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
  // num_order
  if (lhs->num_order != rhs->num_order) {
    return false;
  }
  // num_segment
  if (lhs->num_segment != rhs->num_segment) {
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
  // coef_x
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->coef_x), &(rhs->coef_x)))
  {
    return false;
  }
  // coef_y
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->coef_y), &(rhs->coef_y)))
  {
    return false;
  }
  // coef_z
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->coef_z), &(rhs->coef_z)))
  {
    return false;
  }
  // time
  if (!rosidl_runtime_c__double__Sequence__are_equal(
      &(lhs->time), &(rhs->time)))
  {
    return false;
  }
  // mag_coeff
  if (lhs->mag_coeff != rhs->mag_coeff) {
    return false;
  }
  // order
  if (!rosidl_runtime_c__uint32__Sequence__are_equal(
      &(lhs->order), &(rhs->order)))
  {
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
quadrotor_msgs__msg__PolynomialTrajectory__copy(
  const quadrotor_msgs__msg__PolynomialTrajectory * input,
  quadrotor_msgs__msg__PolynomialTrajectory * output)
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
  // num_order
  output->num_order = input->num_order;
  // num_segment
  output->num_segment = input->num_segment;
  // start_yaw
  output->start_yaw = input->start_yaw;
  // final_yaw
  output->final_yaw = input->final_yaw;
  // coef_x
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->coef_x), &(output->coef_x)))
  {
    return false;
  }
  // coef_y
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->coef_y), &(output->coef_y)))
  {
    return false;
  }
  // coef_z
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->coef_z), &(output->coef_z)))
  {
    return false;
  }
  // time
  if (!rosidl_runtime_c__double__Sequence__copy(
      &(input->time), &(output->time)))
  {
    return false;
  }
  // mag_coeff
  output->mag_coeff = input->mag_coeff;
  // order
  if (!rosidl_runtime_c__uint32__Sequence__copy(
      &(input->order), &(output->order)))
  {
    return false;
  }
  // debug_info
  if (!rosidl_runtime_c__String__copy(
      &(input->debug_info), &(output->debug_info)))
  {
    return false;
  }
  return true;
}

quadrotor_msgs__msg__PolynomialTrajectory *
quadrotor_msgs__msg__PolynomialTrajectory__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PolynomialTrajectory * msg = (quadrotor_msgs__msg__PolynomialTrajectory *)allocator.allocate(sizeof(quadrotor_msgs__msg__PolynomialTrajectory), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__PolynomialTrajectory));
  bool success = quadrotor_msgs__msg__PolynomialTrajectory__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__PolynomialTrajectory__destroy(quadrotor_msgs__msg__PolynomialTrajectory * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__PolynomialTrajectory__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__PolynomialTrajectory__Sequence__init(quadrotor_msgs__msg__PolynomialTrajectory__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PolynomialTrajectory * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__PolynomialTrajectory *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__PolynomialTrajectory), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__PolynomialTrajectory__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__PolynomialTrajectory__fini(&data[i - 1]);
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
quadrotor_msgs__msg__PolynomialTrajectory__Sequence__fini(quadrotor_msgs__msg__PolynomialTrajectory__Sequence * array)
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
      quadrotor_msgs__msg__PolynomialTrajectory__fini(&array->data[i]);
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

quadrotor_msgs__msg__PolynomialTrajectory__Sequence *
quadrotor_msgs__msg__PolynomialTrajectory__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PolynomialTrajectory__Sequence * array = (quadrotor_msgs__msg__PolynomialTrajectory__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__PolynomialTrajectory__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__PolynomialTrajectory__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__PolynomialTrajectory__Sequence__destroy(quadrotor_msgs__msg__PolynomialTrajectory__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__PolynomialTrajectory__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__PolynomialTrajectory__Sequence__are_equal(const quadrotor_msgs__msg__PolynomialTrajectory__Sequence * lhs, const quadrotor_msgs__msg__PolynomialTrajectory__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__PolynomialTrajectory__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__PolynomialTrajectory__Sequence__copy(
  const quadrotor_msgs__msg__PolynomialTrajectory__Sequence * input,
  quadrotor_msgs__msg__PolynomialTrajectory__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__PolynomialTrajectory);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__PolynomialTrajectory * data =
      (quadrotor_msgs__msg__PolynomialTrajectory *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__PolynomialTrajectory__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__PolynomialTrajectory__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__PolynomialTrajectory__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
