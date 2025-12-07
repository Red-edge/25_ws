// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from demo_interfaces_test:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__STRUCT_H_
#define DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'data'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TestMsg in the package demo_interfaces_test.
typedef struct demo_interfaces_test__msg__TestMsg
{
  rosidl_runtime_c__String data;
} demo_interfaces_test__msg__TestMsg;

// Struct for a sequence of demo_interfaces_test__msg__TestMsg.
typedef struct demo_interfaces_test__msg__TestMsg__Sequence
{
  demo_interfaces_test__msg__TestMsg * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} demo_interfaces_test__msg__TestMsg__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__STRUCT_H_
