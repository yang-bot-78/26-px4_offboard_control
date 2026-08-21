// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from race_msgs:msg/GlobalPlannerStatus.idl
// generated code does not contain a copyright notice
#include "race_msgs/msg/detail/global_planner_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `mode`
// Member `reason`
#include "rosidl_runtime_c/string_functions.h"

bool
race_msgs__msg__GlobalPlannerStatus__init(race_msgs__msg__GlobalPlannerStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    race_msgs__msg__GlobalPlannerStatus__fini(msg);
    return false;
  }
  // global_goal_id
  // global_path_id
  // local_goal_seq
  // goal_active
  // final_goal_reached
  // distance_to_final
  // horizontal_speed
  // mode
  if (!rosidl_runtime_c__String__init(&msg->mode)) {
    race_msgs__msg__GlobalPlannerStatus__fini(msg);
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__init(&msg->reason)) {
    race_msgs__msg__GlobalPlannerStatus__fini(msg);
    return false;
  }
  return true;
}

void
race_msgs__msg__GlobalPlannerStatus__fini(race_msgs__msg__GlobalPlannerStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // global_goal_id
  // global_path_id
  // local_goal_seq
  // goal_active
  // final_goal_reached
  // distance_to_final
  // horizontal_speed
  // mode
  rosidl_runtime_c__String__fini(&msg->mode);
  // reason
  rosidl_runtime_c__String__fini(&msg->reason);
}

bool
race_msgs__msg__GlobalPlannerStatus__are_equal(const race_msgs__msg__GlobalPlannerStatus * lhs, const race_msgs__msg__GlobalPlannerStatus * rhs)
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
  // global_goal_id
  if (lhs->global_goal_id != rhs->global_goal_id) {
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
  // goal_active
  if (lhs->goal_active != rhs->goal_active) {
    return false;
  }
  // final_goal_reached
  if (lhs->final_goal_reached != rhs->final_goal_reached) {
    return false;
  }
  // distance_to_final
  if (lhs->distance_to_final != rhs->distance_to_final) {
    return false;
  }
  // horizontal_speed
  if (lhs->horizontal_speed != rhs->horizontal_speed) {
    return false;
  }
  // mode
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->mode), &(rhs->mode)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->reason), &(rhs->reason)))
  {
    return false;
  }
  return true;
}

bool
race_msgs__msg__GlobalPlannerStatus__copy(
  const race_msgs__msg__GlobalPlannerStatus * input,
  race_msgs__msg__GlobalPlannerStatus * output)
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
  // global_goal_id
  output->global_goal_id = input->global_goal_id;
  // global_path_id
  output->global_path_id = input->global_path_id;
  // local_goal_seq
  output->local_goal_seq = input->local_goal_seq;
  // goal_active
  output->goal_active = input->goal_active;
  // final_goal_reached
  output->final_goal_reached = input->final_goal_reached;
  // distance_to_final
  output->distance_to_final = input->distance_to_final;
  // horizontal_speed
  output->horizontal_speed = input->horizontal_speed;
  // mode
  if (!rosidl_runtime_c__String__copy(
      &(input->mode), &(output->mode)))
  {
    return false;
  }
  // reason
  if (!rosidl_runtime_c__String__copy(
      &(input->reason), &(output->reason)))
  {
    return false;
  }
  return true;
}

race_msgs__msg__GlobalPlannerStatus *
race_msgs__msg__GlobalPlannerStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__GlobalPlannerStatus * msg = (race_msgs__msg__GlobalPlannerStatus *)allocator.allocate(sizeof(race_msgs__msg__GlobalPlannerStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(race_msgs__msg__GlobalPlannerStatus));
  bool success = race_msgs__msg__GlobalPlannerStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
race_msgs__msg__GlobalPlannerStatus__destroy(race_msgs__msg__GlobalPlannerStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    race_msgs__msg__GlobalPlannerStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
race_msgs__msg__GlobalPlannerStatus__Sequence__init(race_msgs__msg__GlobalPlannerStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__GlobalPlannerStatus * data = NULL;

  if (size) {
    data = (race_msgs__msg__GlobalPlannerStatus *)allocator.zero_allocate(size, sizeof(race_msgs__msg__GlobalPlannerStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = race_msgs__msg__GlobalPlannerStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        race_msgs__msg__GlobalPlannerStatus__fini(&data[i - 1]);
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
race_msgs__msg__GlobalPlannerStatus__Sequence__fini(race_msgs__msg__GlobalPlannerStatus__Sequence * array)
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
      race_msgs__msg__GlobalPlannerStatus__fini(&array->data[i]);
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

race_msgs__msg__GlobalPlannerStatus__Sequence *
race_msgs__msg__GlobalPlannerStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__GlobalPlannerStatus__Sequence * array = (race_msgs__msg__GlobalPlannerStatus__Sequence *)allocator.allocate(sizeof(race_msgs__msg__GlobalPlannerStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = race_msgs__msg__GlobalPlannerStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
race_msgs__msg__GlobalPlannerStatus__Sequence__destroy(race_msgs__msg__GlobalPlannerStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    race_msgs__msg__GlobalPlannerStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
race_msgs__msg__GlobalPlannerStatus__Sequence__are_equal(const race_msgs__msg__GlobalPlannerStatus__Sequence * lhs, const race_msgs__msg__GlobalPlannerStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!race_msgs__msg__GlobalPlannerStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
race_msgs__msg__GlobalPlannerStatus__Sequence__copy(
  const race_msgs__msg__GlobalPlannerStatus__Sequence * input,
  race_msgs__msg__GlobalPlannerStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(race_msgs__msg__GlobalPlannerStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    race_msgs__msg__GlobalPlannerStatus * data =
      (race_msgs__msg__GlobalPlannerStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!race_msgs__msg__GlobalPlannerStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          race_msgs__msg__GlobalPlannerStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!race_msgs__msg__GlobalPlannerStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
