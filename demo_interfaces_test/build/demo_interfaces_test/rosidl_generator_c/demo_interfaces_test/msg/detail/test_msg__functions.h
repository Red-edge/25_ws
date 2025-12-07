// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from demo_interfaces_test:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__FUNCTIONS_H_
#define DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "demo_interfaces_test/msg/rosidl_generator_c__visibility_control.h"

#include "demo_interfaces_test/msg/detail/test_msg__struct.h"

/// Initialize msg/TestMsg message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * demo_interfaces_test__msg__TestMsg
 * )) before or use
 * demo_interfaces_test__msg__TestMsg__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
bool
demo_interfaces_test__msg__TestMsg__init(demo_interfaces_test__msg__TestMsg * msg);

/// Finalize msg/TestMsg message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
void
demo_interfaces_test__msg__TestMsg__fini(demo_interfaces_test__msg__TestMsg * msg);

/// Create msg/TestMsg message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * demo_interfaces_test__msg__TestMsg__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
demo_interfaces_test__msg__TestMsg *
demo_interfaces_test__msg__TestMsg__create();

/// Destroy msg/TestMsg message.
/**
 * It calls
 * demo_interfaces_test__msg__TestMsg__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
void
demo_interfaces_test__msg__TestMsg__destroy(demo_interfaces_test__msg__TestMsg * msg);

/// Check for msg/TestMsg message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
bool
demo_interfaces_test__msg__TestMsg__are_equal(const demo_interfaces_test__msg__TestMsg * lhs, const demo_interfaces_test__msg__TestMsg * rhs);

/// Copy a msg/TestMsg message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
bool
demo_interfaces_test__msg__TestMsg__copy(
  const demo_interfaces_test__msg__TestMsg * input,
  demo_interfaces_test__msg__TestMsg * output);

/// Initialize array of msg/TestMsg messages.
/**
 * It allocates the memory for the number of elements and calls
 * demo_interfaces_test__msg__TestMsg__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
bool
demo_interfaces_test__msg__TestMsg__Sequence__init(demo_interfaces_test__msg__TestMsg__Sequence * array, size_t size);

/// Finalize array of msg/TestMsg messages.
/**
 * It calls
 * demo_interfaces_test__msg__TestMsg__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
void
demo_interfaces_test__msg__TestMsg__Sequence__fini(demo_interfaces_test__msg__TestMsg__Sequence * array);

/// Create array of msg/TestMsg messages.
/**
 * It allocates the memory for the array and calls
 * demo_interfaces_test__msg__TestMsg__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
demo_interfaces_test__msg__TestMsg__Sequence *
demo_interfaces_test__msg__TestMsg__Sequence__create(size_t size);

/// Destroy array of msg/TestMsg messages.
/**
 * It calls
 * demo_interfaces_test__msg__TestMsg__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
void
demo_interfaces_test__msg__TestMsg__Sequence__destroy(demo_interfaces_test__msg__TestMsg__Sequence * array);

/// Check for msg/TestMsg message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
bool
demo_interfaces_test__msg__TestMsg__Sequence__are_equal(const demo_interfaces_test__msg__TestMsg__Sequence * lhs, const demo_interfaces_test__msg__TestMsg__Sequence * rhs);

/// Copy an array of msg/TestMsg messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_demo_interfaces_test
bool
demo_interfaces_test__msg__TestMsg__Sequence__copy(
  const demo_interfaces_test__msg__TestMsg__Sequence * input,
  demo_interfaces_test__msg__TestMsg__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__FUNCTIONS_H_
