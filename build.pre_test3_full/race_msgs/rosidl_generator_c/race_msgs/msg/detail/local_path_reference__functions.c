// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from race_msgs:msg/LocalPathReference.idl
// generated code does not contain a copyright notice
#include "race_msgs/msg/detail/local_path_reference__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `local_goal`
// Member `points`
// Member `continuation_points`
#include "geometry_msgs/msg/detail/point__functions.h"

bool
race_msgs__msg__LocalPathReference__init(race_msgs__msg__LocalPathReference * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    race_msgs__msg__LocalPathReference__fini(msg);
    return false;
  }
  // global_path_id
  // local_goal_seq
  // local_goal
  if (!geometry_msgs__msg__Point__init(&msg->local_goal)) {
    race_msgs__msg__LocalPathReference__fini(msg);
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->points, 0)) {
    race_msgs__msg__LocalPathReference__fini(msg);
    return false;
  }
  // continuation_points
  if (!geometry_msgs__msg__Point__Sequence__init(&msg->continuation_points, 0)) {
    race_msgs__msg__LocalPathReference__fini(msg);
    return false;
  }
  // arc_length
  // minimum_clearance
  return true;
}

void
race_msgs__msg__LocalPathReference__fini(race_msgs__msg__LocalPathReference * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // global_path_id
  // local_goal_seq
  // local_goal
  geometry_msgs__msg__Point__fini(&msg->local_goal);
  // points
  geometry_msgs__msg__Point__Sequence__fini(&msg->points);
  // continuation_points
  geometry_msgs__msg__Point__Sequence__fini(&msg->continuation_points);
  // arc_length
  // minimum_clearance
}

bool
race_msgs__msg__LocalPathReference__are_equal(const race_msgs__msg__LocalPathReference * lhs, const race_msgs__msg__LocalPathReference * rhs)
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
  // global_path_id
  if (lhs->global_path_id != rhs->global_path_id) {
    return false;
  }
  // local_goal_seq
  if (lhs->local_goal_seq != rhs->local_goal_seq) {
    return false;
  }
  // local_goal
  if (!geometry_msgs__msg__Point__are_equal(
      &(lhs->local_goal), &(rhs->local_goal)))
  {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->points), &(rhs->points)))
  {
    return false;
  }
  // continuation_points
  if (!geometry_msgs__msg__Point__Sequence__are_equal(
      &(lhs->continuation_points), &(rhs->continuation_points)))
  {
    return false;
  }
  // arc_length
  if (lhs->arc_length != rhs->arc_length) {
    return false;
  }
  // minimum_clearance
  if (lhs->minimum_clearance != rhs->minimum_clearance) {
    return false;
  }
  return true;
}

bool
race_msgs__msg__LocalPathReference__copy(
  const race_msgs__msg__LocalPathReference * input,
  race_msgs__msg__LocalPathReference * output)
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
  // global_path_id
  output->global_path_id = input->global_path_id;
  // local_goal_seq
  output->local_goal_seq = input->local_goal_seq;
  // local_goal
  if (!geometry_msgs__msg__Point__copy(
      &(input->local_goal), &(output->local_goal)))
  {
    return false;
  }
  // points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->points), &(output->points)))
  {
    return false;
  }
  // continuation_points
  if (!geometry_msgs__msg__Point__Sequence__copy(
      &(input->continuation_points), &(output->continuation_points)))
  {
    return false;
  }
  // arc_length
  output->arc_length = input->arc_length;
  // minimum_clearance
  output->minimum_clearance = input->minimum_clearance;
  return true;
}

race_msgs__msg__LocalPathReference *
race_msgs__msg__LocalPathReference__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__LocalPathReference * msg = (race_msgs__msg__LocalPathReference *)allocator.allocate(sizeof(race_msgs__msg__LocalPathReference), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(race_msgs__msg__LocalPathReference));
  bool success = race_msgs__msg__LocalPathReference__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
race_msgs__msg__LocalPathReference__destroy(race_msgs__msg__LocalPathReference * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    race_msgs__msg__LocalPathReference__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
race_msgs__msg__LocalPathReference__Sequence__init(race_msgs__msg__LocalPathReference__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__LocalPathReference * data = NULL;

  if (size) {
    data = (race_msgs__msg__LocalPathReference *)allocator.zero_allocate(size, sizeof(race_msgs__msg__LocalPathReference), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = race_msgs__msg__LocalPathReference__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        race_msgs__msg__LocalPathReference__fini(&data[i - 1]);
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
race_msgs__msg__LocalPathReference__Sequence__fini(race_msgs__msg__LocalPathReference__Sequence * array)
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
      race_msgs__msg__LocalPathReference__fini(&array->data[i]);
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

race_msgs__msg__LocalPathReference__Sequence *
race_msgs__msg__LocalPathReference__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__LocalPathReference__Sequence * array = (race_msgs__msg__LocalPathReference__Sequence *)allocator.allocate(sizeof(race_msgs__msg__LocalPathReference__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = race_msgs__msg__LocalPathReference__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
race_msgs__msg__LocalPathReference__Sequence__destroy(race_msgs__msg__LocalPathReference__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    race_msgs__msg__LocalPathReference__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
race_msgs__msg__LocalPathReference__Sequence__are_equal(const race_msgs__msg__LocalPathReference__Sequence * lhs, const race_msgs__msg__LocalPathReference__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!race_msgs__msg__LocalPathReference__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
race_msgs__msg__LocalPathReference__Sequence__copy(
  const race_msgs__msg__LocalPathReference__Sequence * input,
  race_msgs__msg__LocalPathReference__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(race_msgs__msg__LocalPathReference);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    race_msgs__msg__LocalPathReference * data =
      (race_msgs__msg__LocalPathReference *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!race_msgs__msg__LocalPathReference__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          race_msgs__msg__LocalPathReference__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!race_msgs__msg__LocalPathReference__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
