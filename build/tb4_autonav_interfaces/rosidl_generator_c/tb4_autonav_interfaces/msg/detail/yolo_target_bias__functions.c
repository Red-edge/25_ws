// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice
#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

bool
tb4_autonav_interfaces__msg__YoloTargetBias__init(tb4_autonav_interfaces__msg__YoloTargetBias * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    tb4_autonav_interfaces__msg__YoloTargetBias__fini(msg);
    return false;
  }
  // has_target
  // type
  if (!rosidl_runtime_c__String__init(&msg->type)) {
    tb4_autonav_interfaces__msg__YoloTargetBias__fini(msg);
    return false;
  }
  // distance_m
  // u_norm
  // v_norm
  return true;
}

void
tb4_autonav_interfaces__msg__YoloTargetBias__fini(tb4_autonav_interfaces__msg__YoloTargetBias * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // has_target
  // type
  rosidl_runtime_c__String__fini(&msg->type);
  // distance_m
  // u_norm
  // v_norm
}

bool
tb4_autonav_interfaces__msg__YoloTargetBias__are_equal(const tb4_autonav_interfaces__msg__YoloTargetBias * lhs, const tb4_autonav_interfaces__msg__YoloTargetBias * rhs)
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
  // has_target
  if (lhs->has_target != rhs->has_target) {
    return false;
  }
  // type
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->type), &(rhs->type)))
  {
    return false;
  }
  // distance_m
  if (lhs->distance_m != rhs->distance_m) {
    return false;
  }
  // u_norm
  if (lhs->u_norm != rhs->u_norm) {
    return false;
  }
  // v_norm
  if (lhs->v_norm != rhs->v_norm) {
    return false;
  }
  return true;
}

bool
tb4_autonav_interfaces__msg__YoloTargetBias__copy(
  const tb4_autonav_interfaces__msg__YoloTargetBias * input,
  tb4_autonav_interfaces__msg__YoloTargetBias * output)
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
  // has_target
  output->has_target = input->has_target;
  // type
  if (!rosidl_runtime_c__String__copy(
      &(input->type), &(output->type)))
  {
    return false;
  }
  // distance_m
  output->distance_m = input->distance_m;
  // u_norm
  output->u_norm = input->u_norm;
  // v_norm
  output->v_norm = input->v_norm;
  return true;
}

tb4_autonav_interfaces__msg__YoloTargetBias *
tb4_autonav_interfaces__msg__YoloTargetBias__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tb4_autonav_interfaces__msg__YoloTargetBias * msg = (tb4_autonav_interfaces__msg__YoloTargetBias *)allocator.allocate(sizeof(tb4_autonav_interfaces__msg__YoloTargetBias), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(tb4_autonav_interfaces__msg__YoloTargetBias));
  bool success = tb4_autonav_interfaces__msg__YoloTargetBias__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
tb4_autonav_interfaces__msg__YoloTargetBias__destroy(tb4_autonav_interfaces__msg__YoloTargetBias * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    tb4_autonav_interfaces__msg__YoloTargetBias__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__init(tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tb4_autonav_interfaces__msg__YoloTargetBias * data = NULL;

  if (size) {
    data = (tb4_autonav_interfaces__msg__YoloTargetBias *)allocator.zero_allocate(size, sizeof(tb4_autonav_interfaces__msg__YoloTargetBias), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = tb4_autonav_interfaces__msg__YoloTargetBias__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        tb4_autonav_interfaces__msg__YoloTargetBias__fini(&data[i - 1]);
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
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__fini(tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * array)
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
      tb4_autonav_interfaces__msg__YoloTargetBias__fini(&array->data[i]);
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

tb4_autonav_interfaces__msg__YoloTargetBias__Sequence *
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * array = (tb4_autonav_interfaces__msg__YoloTargetBias__Sequence *)allocator.allocate(sizeof(tb4_autonav_interfaces__msg__YoloTargetBias__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__destroy(tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__are_equal(const tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * lhs, const tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!tb4_autonav_interfaces__msg__YoloTargetBias__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__copy(
  const tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * input,
  tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(tb4_autonav_interfaces__msg__YoloTargetBias);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    tb4_autonav_interfaces__msg__YoloTargetBias * data =
      (tb4_autonav_interfaces__msg__YoloTargetBias *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!tb4_autonav_interfaces__msg__YoloTargetBias__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          tb4_autonav_interfaces__msg__YoloTargetBias__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!tb4_autonav_interfaces__msg__YoloTargetBias__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
