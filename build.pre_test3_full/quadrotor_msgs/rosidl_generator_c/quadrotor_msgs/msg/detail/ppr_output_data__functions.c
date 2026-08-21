// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from quadrotor_msgs:msg/PPROutputData.idl
// generated code does not contain a copyright notice
#include "quadrotor_msgs/msg/detail/ppr_output_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
quadrotor_msgs__msg__PPROutputData__init(quadrotor_msgs__msg__PPROutputData * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    quadrotor_msgs__msg__PPROutputData__fini(msg);
    return false;
  }
  // quad_time
  // des_thrust
  // des_roll
  // des_pitch
  // des_yaw
  // est_roll
  // est_pitch
  // est_yaw
  // est_angvel_x
  // est_angvel_y
  // est_angvel_z
  // est_acc_x
  // est_acc_y
  // est_acc_z
  // pwm
  return true;
}

void
quadrotor_msgs__msg__PPROutputData__fini(quadrotor_msgs__msg__PPROutputData * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // quad_time
  // des_thrust
  // des_roll
  // des_pitch
  // des_yaw
  // est_roll
  // est_pitch
  // est_yaw
  // est_angvel_x
  // est_angvel_y
  // est_angvel_z
  // est_acc_x
  // est_acc_y
  // est_acc_z
  // pwm
}

bool
quadrotor_msgs__msg__PPROutputData__are_equal(const quadrotor_msgs__msg__PPROutputData * lhs, const quadrotor_msgs__msg__PPROutputData * rhs)
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
  // quad_time
  if (lhs->quad_time != rhs->quad_time) {
    return false;
  }
  // des_thrust
  if (lhs->des_thrust != rhs->des_thrust) {
    return false;
  }
  // des_roll
  if (lhs->des_roll != rhs->des_roll) {
    return false;
  }
  // des_pitch
  if (lhs->des_pitch != rhs->des_pitch) {
    return false;
  }
  // des_yaw
  if (lhs->des_yaw != rhs->des_yaw) {
    return false;
  }
  // est_roll
  if (lhs->est_roll != rhs->est_roll) {
    return false;
  }
  // est_pitch
  if (lhs->est_pitch != rhs->est_pitch) {
    return false;
  }
  // est_yaw
  if (lhs->est_yaw != rhs->est_yaw) {
    return false;
  }
  // est_angvel_x
  if (lhs->est_angvel_x != rhs->est_angvel_x) {
    return false;
  }
  // est_angvel_y
  if (lhs->est_angvel_y != rhs->est_angvel_y) {
    return false;
  }
  // est_angvel_z
  if (lhs->est_angvel_z != rhs->est_angvel_z) {
    return false;
  }
  // est_acc_x
  if (lhs->est_acc_x != rhs->est_acc_x) {
    return false;
  }
  // est_acc_y
  if (lhs->est_acc_y != rhs->est_acc_y) {
    return false;
  }
  // est_acc_z
  if (lhs->est_acc_z != rhs->est_acc_z) {
    return false;
  }
  // pwm
  for (size_t i = 0; i < 4; ++i) {
    if (lhs->pwm[i] != rhs->pwm[i]) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__PPROutputData__copy(
  const quadrotor_msgs__msg__PPROutputData * input,
  quadrotor_msgs__msg__PPROutputData * output)
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
  // quad_time
  output->quad_time = input->quad_time;
  // des_thrust
  output->des_thrust = input->des_thrust;
  // des_roll
  output->des_roll = input->des_roll;
  // des_pitch
  output->des_pitch = input->des_pitch;
  // des_yaw
  output->des_yaw = input->des_yaw;
  // est_roll
  output->est_roll = input->est_roll;
  // est_pitch
  output->est_pitch = input->est_pitch;
  // est_yaw
  output->est_yaw = input->est_yaw;
  // est_angvel_x
  output->est_angvel_x = input->est_angvel_x;
  // est_angvel_y
  output->est_angvel_y = input->est_angvel_y;
  // est_angvel_z
  output->est_angvel_z = input->est_angvel_z;
  // est_acc_x
  output->est_acc_x = input->est_acc_x;
  // est_acc_y
  output->est_acc_y = input->est_acc_y;
  // est_acc_z
  output->est_acc_z = input->est_acc_z;
  // pwm
  for (size_t i = 0; i < 4; ++i) {
    output->pwm[i] = input->pwm[i];
  }
  return true;
}

quadrotor_msgs__msg__PPROutputData *
quadrotor_msgs__msg__PPROutputData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PPROutputData * msg = (quadrotor_msgs__msg__PPROutputData *)allocator.allocate(sizeof(quadrotor_msgs__msg__PPROutputData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(quadrotor_msgs__msg__PPROutputData));
  bool success = quadrotor_msgs__msg__PPROutputData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
quadrotor_msgs__msg__PPROutputData__destroy(quadrotor_msgs__msg__PPROutputData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    quadrotor_msgs__msg__PPROutputData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
quadrotor_msgs__msg__PPROutputData__Sequence__init(quadrotor_msgs__msg__PPROutputData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PPROutputData * data = NULL;

  if (size) {
    data = (quadrotor_msgs__msg__PPROutputData *)allocator.zero_allocate(size, sizeof(quadrotor_msgs__msg__PPROutputData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = quadrotor_msgs__msg__PPROutputData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        quadrotor_msgs__msg__PPROutputData__fini(&data[i - 1]);
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
quadrotor_msgs__msg__PPROutputData__Sequence__fini(quadrotor_msgs__msg__PPROutputData__Sequence * array)
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
      quadrotor_msgs__msg__PPROutputData__fini(&array->data[i]);
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

quadrotor_msgs__msg__PPROutputData__Sequence *
quadrotor_msgs__msg__PPROutputData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  quadrotor_msgs__msg__PPROutputData__Sequence * array = (quadrotor_msgs__msg__PPROutputData__Sequence *)allocator.allocate(sizeof(quadrotor_msgs__msg__PPROutputData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = quadrotor_msgs__msg__PPROutputData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
quadrotor_msgs__msg__PPROutputData__Sequence__destroy(quadrotor_msgs__msg__PPROutputData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    quadrotor_msgs__msg__PPROutputData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
quadrotor_msgs__msg__PPROutputData__Sequence__are_equal(const quadrotor_msgs__msg__PPROutputData__Sequence * lhs, const quadrotor_msgs__msg__PPROutputData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!quadrotor_msgs__msg__PPROutputData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
quadrotor_msgs__msg__PPROutputData__Sequence__copy(
  const quadrotor_msgs__msg__PPROutputData__Sequence * input,
  quadrotor_msgs__msg__PPROutputData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(quadrotor_msgs__msg__PPROutputData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    quadrotor_msgs__msg__PPROutputData * data =
      (quadrotor_msgs__msg__PPROutputData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!quadrotor_msgs__msg__PPROutputData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          quadrotor_msgs__msg__PPROutputData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!quadrotor_msgs__msg__PPROutputData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
