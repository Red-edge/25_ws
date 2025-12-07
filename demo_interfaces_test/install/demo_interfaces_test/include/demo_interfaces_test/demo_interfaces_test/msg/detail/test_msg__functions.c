// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from demo_interfaces_test:msg/TestMsg.idl
// generated code does not contain a copyright notice
#include "demo_interfaces_test/msg/detail/test_msg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `data`
#include "rosidl_runtime_c/string_functions.h"

bool
demo_interfaces_test__msg__TestMsg__init(demo_interfaces_test__msg__TestMsg * msg)
{
  if (!msg) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__init(&msg->data)) {
    demo_interfaces_test__msg__TestMsg__fini(msg);
    return false;
  }
  return true;
}

void
demo_interfaces_test__msg__TestMsg__fini(demo_interfaces_test__msg__TestMsg * msg)
{
  if (!msg) {
    return;
  }
  // data
  rosidl_runtime_c__String__fini(&msg->data);
}

bool
demo_interfaces_test__msg__TestMsg__are_equal(const demo_interfaces_test__msg__TestMsg * lhs, const demo_interfaces_test__msg__TestMsg * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->data), &(rhs->data)))
  {
    return false;
  }
  return true;
}

bool
demo_interfaces_test__msg__TestMsg__copy(
  const demo_interfaces_test__msg__TestMsg * input,
  demo_interfaces_test__msg__TestMsg * output)
{
  if (!input || !output) {
    return false;
  }
  // data
  if (!rosidl_runtime_c__String__copy(
      &(input->data), &(output->data)))
  {
    return false;
  }
  return true;
}

demo_interfaces_test__msg__TestMsg *
demo_interfaces_test__msg__TestMsg__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  demo_interfaces_test__msg__TestMsg * msg = (demo_interfaces_test__msg__TestMsg *)allocator.allocate(sizeof(demo_interfaces_test__msg__TestMsg), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(demo_interfaces_test__msg__TestMsg));
  bool success = demo_interfaces_test__msg__TestMsg__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
demo_interfaces_test__msg__TestMsg__destroy(demo_interfaces_test__msg__TestMsg * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    demo_interfaces_test__msg__TestMsg__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
demo_interfaces_test__msg__TestMsg__Sequence__init(demo_interfaces_test__msg__TestMsg__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  demo_interfaces_test__msg__TestMsg * data = NULL;

  if (size) {
    data = (demo_interfaces_test__msg__TestMsg *)allocator.zero_allocate(size, sizeof(demo_interfaces_test__msg__TestMsg), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = demo_interfaces_test__msg__TestMsg__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        demo_interfaces_test__msg__TestMsg__fini(&data[i - 1]);
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
demo_interfaces_test__msg__TestMsg__Sequence__fini(demo_interfaces_test__msg__TestMsg__Sequence * array)
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
      demo_interfaces_test__msg__TestMsg__fini(&array->data[i]);
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

demo_interfaces_test__msg__TestMsg__Sequence *
demo_interfaces_test__msg__TestMsg__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  demo_interfaces_test__msg__TestMsg__Sequence * array = (demo_interfaces_test__msg__TestMsg__Sequence *)allocator.allocate(sizeof(demo_interfaces_test__msg__TestMsg__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = demo_interfaces_test__msg__TestMsg__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
demo_interfaces_test__msg__TestMsg__Sequence__destroy(demo_interfaces_test__msg__TestMsg__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    demo_interfaces_test__msg__TestMsg__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
demo_interfaces_test__msg__TestMsg__Sequence__are_equal(const demo_interfaces_test__msg__TestMsg__Sequence * lhs, const demo_interfaces_test__msg__TestMsg__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!demo_interfaces_test__msg__TestMsg__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
demo_interfaces_test__msg__TestMsg__Sequence__copy(
  const demo_interfaces_test__msg__TestMsg__Sequence * input,
  demo_interfaces_test__msg__TestMsg__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(demo_interfaces_test__msg__TestMsg);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    demo_interfaces_test__msg__TestMsg * data =
      (demo_interfaces_test__msg__TestMsg *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!demo_interfaces_test__msg__TestMsg__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          demo_interfaces_test__msg__TestMsg__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!demo_interfaces_test__msg__TestMsg__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
