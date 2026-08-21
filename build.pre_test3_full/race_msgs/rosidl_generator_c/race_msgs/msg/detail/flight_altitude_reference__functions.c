// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice
#include "race_msgs/msg/detail/flight_altitude_reference__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
race_msgs__msg__FlightAltitudeReference__init(race_msgs__msg__FlightAltitudeReference * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    race_msgs__msg__FlightAltitudeReference__fini(msg);
    return false;
  }
  // flight_id
  // valid
  // target_agl_m
  // min_agl_m
  // max_agl_m
  // ground_z_local_ned
  // target_z_local_ned
  // ground_z_map
  // target_z_map
  return true;
}

void
race_msgs__msg__FlightAltitudeReference__fini(race_msgs__msg__FlightAltitudeReference * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // flight_id
  // valid
  // target_agl_m
  // min_agl_m
  // max_agl_m
  // ground_z_local_ned
  // target_z_local_ned
  // ground_z_map
  // target_z_map
}

bool
race_msgs__msg__FlightAltitudeReference__are_equal(const race_msgs__msg__FlightAltitudeReference * lhs, const race_msgs__msg__FlightAltitudeReference * rhs)
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
  // flight_id
  if (lhs->flight_id != rhs->flight_id) {
    return false;
  }
  // valid
  if (lhs->valid != rhs->valid) {
    return false;
  }
  // target_agl_m
  if (lhs->target_agl_m != rhs->target_agl_m) {
    return false;
  }
  // min_agl_m
  if (lhs->min_agl_m != rhs->min_agl_m) {
    return false;
  }
  // max_agl_m
  if (lhs->max_agl_m != rhs->max_agl_m) {
    return false;
  }
  // ground_z_local_ned
  if (lhs->ground_z_local_ned != rhs->ground_z_local_ned) {
    return false;
  }
  // target_z_local_ned
  if (lhs->target_z_local_ned != rhs->target_z_local_ned) {
    return false;
  }
  // ground_z_map
  if (lhs->ground_z_map != rhs->ground_z_map) {
    return false;
  }
  // target_z_map
  if (lhs->target_z_map != rhs->target_z_map) {
    return false;
  }
  return true;
}

bool
race_msgs__msg__FlightAltitudeReference__copy(
  const race_msgs__msg__FlightAltitudeReference * input,
  race_msgs__msg__FlightAltitudeReference * output)
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
  // flight_id
  output->flight_id = input->flight_id;
  // valid
  output->valid = input->valid;
  // target_agl_m
  output->target_agl_m = input->target_agl_m;
  // min_agl_m
  output->min_agl_m = input->min_agl_m;
  // max_agl_m
  output->max_agl_m = input->max_agl_m;
  // ground_z_local_ned
  output->ground_z_local_ned = input->ground_z_local_ned;
  // target_z_local_ned
  output->target_z_local_ned = input->target_z_local_ned;
  // ground_z_map
  output->ground_z_map = input->ground_z_map;
  // target_z_map
  output->target_z_map = input->target_z_map;
  return true;
}

race_msgs__msg__FlightAltitudeReference *
race_msgs__msg__FlightAltitudeReference__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__FlightAltitudeReference * msg = (race_msgs__msg__FlightAltitudeReference *)allocator.allocate(sizeof(race_msgs__msg__FlightAltitudeReference), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(race_msgs__msg__FlightAltitudeReference));
  bool success = race_msgs__msg__FlightAltitudeReference__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
race_msgs__msg__FlightAltitudeReference__destroy(race_msgs__msg__FlightAltitudeReference * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    race_msgs__msg__FlightAltitudeReference__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
race_msgs__msg__FlightAltitudeReference__Sequence__init(race_msgs__msg__FlightAltitudeReference__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__FlightAltitudeReference * data = NULL;

  if (size) {
    data = (race_msgs__msg__FlightAltitudeReference *)allocator.zero_allocate(size, sizeof(race_msgs__msg__FlightAltitudeReference), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = race_msgs__msg__FlightAltitudeReference__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        race_msgs__msg__FlightAltitudeReference__fini(&data[i - 1]);
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
race_msgs__msg__FlightAltitudeReference__Sequence__fini(race_msgs__msg__FlightAltitudeReference__Sequence * array)
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
      race_msgs__msg__FlightAltitudeReference__fini(&array->data[i]);
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

race_msgs__msg__FlightAltitudeReference__Sequence *
race_msgs__msg__FlightAltitudeReference__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  race_msgs__msg__FlightAltitudeReference__Sequence * array = (race_msgs__msg__FlightAltitudeReference__Sequence *)allocator.allocate(sizeof(race_msgs__msg__FlightAltitudeReference__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = race_msgs__msg__FlightAltitudeReference__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
race_msgs__msg__FlightAltitudeReference__Sequence__destroy(race_msgs__msg__FlightAltitudeReference__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    race_msgs__msg__FlightAltitudeReference__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
race_msgs__msg__FlightAltitudeReference__Sequence__are_equal(const race_msgs__msg__FlightAltitudeReference__Sequence * lhs, const race_msgs__msg__FlightAltitudeReference__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!race_msgs__msg__FlightAltitudeReference__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
race_msgs__msg__FlightAltitudeReference__Sequence__copy(
  const race_msgs__msg__FlightAltitudeReference__Sequence * input,
  race_msgs__msg__FlightAltitudeReference__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(race_msgs__msg__FlightAltitudeReference);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    race_msgs__msg__FlightAltitudeReference * data =
      (race_msgs__msg__FlightAltitudeReference *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!race_msgs__msg__FlightAltitudeReference__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          race_msgs__msg__FlightAltitudeReference__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!race_msgs__msg__FlightAltitudeReference__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
