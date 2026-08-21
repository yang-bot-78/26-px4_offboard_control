// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from race_msgs:msg/NavigationSetpoint.idl
// generated code does not contain a copyright notice
#include "race_msgs/msg/detail/navigation_setpoint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `position`
#include "geometry_msgs/msg/detail/point__functions.h"
// Member `velocity`
#include "geometry_msgs/msg/detail/vector3__functions.h"

bool
race_msgs__msg__NavigationSetpoint__init(race_msgs__msg__NavigationSetpoint * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    race_msgs__msg__NavigationSetpoint__fini(msg);
    return false;
  }
  // position
  if (!geometry_msgs__msg__Point__init(&msg->position)) {
    race_msgs__msg__NavigationSetpoint__fini(msg);
    return false;
  }
  // velocity_valid
  // velocity
  if (!geometry_msgs__msg__Vector3__init(&msg->velocity)) {
    race_msgs__msg__NavigationSetpoint__fini(msg);
    return false;
  }
  // yaw
  // yaw_rate_valid
  // yaw_rate
  return true;
}

void
race_msgs__msg__NavigationSetpoint__fini(race_msgs__msg__NavigationSetpoint * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // position
  geometry_msgs__msg__Point__fini(&msg->position);
  // velocity_valid
  // velocity
  geometry_msgs__msg__Vector3__fini(&msg->velocity);
  // yaw
  // yaw_rate_valid
  // yaw_rate
}

bool
race_msgs__msg__NavigationSetpoint__are_equal(const race_msgs__msg__NavigationSetpoint * lhs, const race_msgs__msg__NavigationSetpoint * rhs)
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
  // position
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->position), &(rhs->position)))
  {
    return false;
  }
  // velocity_valid
  if (lhs->velocity_valid != rhs->velocity_valid) {
    return false;
  }
  // velocity
  if (!geometry_msgs__msg__Vector3__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  // yaw_rate_valid
  if (lhs->yaw_rate_valid != rhs->yaw_rate_valid) {
    return false;
  }
  // yaw_rate
  if (lhs->yaw_rate != rhs->yaw_rate) {
    return false;
  }
  return true;
}

bool
race_msgs__msg__NavigationSetpoint__copy(
  const race_msgs__msg__NavigationSetpoint * input,
  race_msgs__msg__NavigationSetpoint * output)
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
  // position
  if (!geometry_msgs__msg__Point__copy(
      &(input->position), &(output->position)))
  {
    return false;
  }
  // velocity_valid
  output->velocity_valid = input->velocity_valid;
  // velocity
  if (!geometry_msgs__msg__Vector3__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  // yaw
  output->yaw = input->yaw;
  // yaw_rate_valid
  output->yaw_rate_valid = input->yaw_rate_valid;
  // yaw_rate
  output->yaw_rate = input->yaw_rate;
  return true;
}

race_msgs__msg__NavigationSetpoint *
race_msgs__msg__NavigationSetpoint__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__NavigationSetpoint * msg = (race_msgs__msg__NavigationSetpoint *)allocator.allocate(sizeof(race_msgs__msg__NavigationSetpoint), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(race_msgs__msg__NavigationSetpoint));
  bool success = race_msgs__msg__NavigationSetpoint__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
race_msgs__msg__NavigationSetpoint__destroy(race_msgs__msg__NavigationSetpoint * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    race_msgs__msg__NavigationSetpoint__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
race_msgs__msg__NavigationSetpoint__Sequence__init(race_msgs__msg__NavigationSetpoint__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__NavigationSetpoint * data = NULL;

  if (size) {
    data = (race_msgs__msg__NavigationSetpoint *)allocator.zero_allocate(size, sizeof(race_msgs__msg__NavigationSetpoint), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = race_msgs__msg__NavigationSetpoint__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        race_msgs__msg__NavigationSetpoint__fini(&data[i - 1]);
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
race_msgs__msg__NavigationSetpoint__Sequence__fini(race_msgs__msg__NavigationSetpoint__Sequence * array)
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
      race_msgs__msg__NavigationSetpoint__fini(&array->data[i]);
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

race_msgs__msg__NavigationSetpoint__Sequence *
race_msgs__msg__NavigationSetpoint__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__NavigationSetpoint__Sequence * array = (race_msgs__msg__NavigationSetpoint__Sequence *)allocator.allocate(sizeof(race_msgs__msg__NavigationSetpoint__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = race_msgs__msg__NavigationSetpoint__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
race_msgs__msg__NavigationSetpoint__Sequence__destroy(race_msgs__msg__NavigationSetpoint__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    race_msgs__msg__NavigationSetpoint__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
race_msgs__msg__NavigationSetpoint__Sequence__are_equal(const race_msgs__msg__NavigationSetpoint__Sequence * lhs, const race_msgs__msg__NavigationSetpoint__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!race_msgs__msg__NavigationSetpoint__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
race_msgs__msg__NavigationSetpoint__Sequence__copy(
  const race_msgs__msg__NavigationSetpoint__Sequence * input,
  race_msgs__msg__NavigationSetpoint__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(race_msgs__msg__NavigationSetpoint);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    race_msgs__msg__NavigationSetpoint * data =
      (race_msgs__msg__NavigationSetpoint *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!race_msgs__msg__NavigationSetpoint__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          race_msgs__msg__NavigationSetpoint__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!race_msgs__msg__NavigationSetpoint__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
