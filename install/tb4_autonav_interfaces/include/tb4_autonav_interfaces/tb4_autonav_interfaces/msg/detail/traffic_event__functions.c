// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tb4_autonav_interfaces:msg/TrafficEvent.idl
// generated code does not contain a copyright notice
#include "tb4_autonav_interfaces/msg/detail/traffic_event__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `stamp`
#include "builtin_interfaces/msg/detail/time__functions.h"
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

bool
tb4_autonav_interfaces__msg__TrafficEvent__init(tb4_autonav_interfaces__msg__TrafficEvent * msg)
{
  if (!msg) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__init(&msg->stamp)) {
    tb4_autonav_interfaces__msg__TrafficEvent__fini(msg);
    return false;
  }
  // is_ready
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    tb4_autonav_interfaces__msg__TrafficEvent__fini(msg);
    return false;
  }
  // distance
  return true;
}

void
tb4_autonav_interfaces__msg__TrafficEvent__fini(tb4_autonav_interfaces__msg__TrafficEvent * msg)
{
  if (!msg) {
    return;
  }
  // stamp
  builtin_interfaces__msg__Time__fini(&msg->stamp);
  // is_ready
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // distance
}

bool
tb4_autonav_interfaces__msg__TrafficEvent__are_equal(const tb4_autonav_interfaces__msg__TrafficEvent * lhs, const tb4_autonav_interfaces__msg__TrafficEvent * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__are_equal(
      &(lhs->stamp), &(rhs->stamp)))
  {
    return false;
  }
  // is_ready
  if (lhs->is_ready != rhs->is_ready) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // distance
  if (lhs->distance != rhs->distance) {
    return false;
  }
  return true;
}

bool
tb4_autonav_interfaces__msg__TrafficEvent__copy(
  const tb4_autonav_interfaces__msg__TrafficEvent * input,
  tb4_autonav_interfaces__msg__TrafficEvent * output)
{
  if (!input || !output) {
    return false;
  }
  // stamp
  if (!builtin_interfaces__msg__Time__copy(
      &(input->stamp), &(output->stamp)))
  {
    return false;
  }
  // is_ready
  output->is_ready = input->is_ready;
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // distance
  output->distance = input->distance;
  return true;
}

tb4_autonav_interfaces__msg__TrafficEvent *
tb4_autonav_interfaces__msg__TrafficEvent__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tb4_autonav_interfaces__msg__TrafficEvent * msg = (tb4_autonav_interfaces__msg__TrafficEvent *)allocator.allocate(sizeof(tb4_autonav_interfaces__msg__TrafficEvent), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tb4_autonav_interfaces__msg__TrafficEvent));
  bool success = tb4_autonav_interfaces__msg__TrafficEvent__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tb4_autonav_interfaces__msg__TrafficEvent__destroy(tb4_autonav_interfaces__msg__TrafficEvent * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tb4_autonav_interfaces__msg__TrafficEvent__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tb4_autonav_interfaces__msg__TrafficEvent__Sequence__init(tb4_autonav_interfaces__msg__TrafficEvent__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tb4_autonav_interfaces__msg__TrafficEvent * data = NULL;

  if (size) {
    data = (tb4_autonav_interfaces__msg__TrafficEvent *)allocator.zero_allocate(size, sizeof(tb4_autonav_interfaces__msg__TrafficEvent), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tb4_autonav_interfaces__msg__TrafficEvent__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tb4_autonav_interfaces__msg__TrafficEvent__fini(&data[i - 1]);
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
tb4_autonav_interfaces__msg__TrafficEvent__Sequence__fini(tb4_autonav_interfaces__msg__TrafficEvent__Sequence * array)
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
      tb4_autonav_interfaces__msg__TrafficEvent__fini(&array->data[i]);
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

tb4_autonav_interfaces__msg__TrafficEvent__Sequence *
tb4_autonav_interfaces__msg__TrafficEvent__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tb4_autonav_interfaces__msg__TrafficEvent__Sequence * array = (tb4_autonav_interfaces__msg__TrafficEvent__Sequence *)allocator.allocate(sizeof(tb4_autonav_interfaces__msg__TrafficEvent__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tb4_autonav_interfaces__msg__TrafficEvent__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tb4_autonav_interfaces__msg__TrafficEvent__Sequence__destroy(tb4_autonav_interfaces__msg__TrafficEvent__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tb4_autonav_interfaces__msg__TrafficEvent__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tb4_autonav_interfaces__msg__TrafficEvent__Sequence__are_equal(const tb4_autonav_interfaces__msg__TrafficEvent__Sequence * lhs, const tb4_autonav_interfaces__msg__TrafficEvent__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tb4_autonav_interfaces__msg__TrafficEvent__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tb4_autonav_interfaces__msg__TrafficEvent__Sequence__copy(
  const tb4_autonav_interfaces__msg__TrafficEvent__Sequence * input,
  tb4_autonav_interfaces__msg__TrafficEvent__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tb4_autonav_interfaces__msg__TrafficEvent);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tb4_autonav_interfaces__msg__TrafficEvent * data =
      (tb4_autonav_interfaces__msg__TrafficEvent *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tb4_autonav_interfaces__msg__TrafficEvent__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tb4_autonav_interfaces__msg__TrafficEvent__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tb4_autonav_interfaces__msg__TrafficEvent__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
