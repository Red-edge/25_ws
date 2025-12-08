// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__FUNCTIONS_H_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "tb4_autonav_interfaces/msg/rosidl_generator_c__visibility_control.h"

#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__struct.h"

/// Initialize msg/YoloTargetBias message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * tb4_autonav_interfaces__msg__YoloTargetBias
 * )) before or use
 * tb4_autonav_interfaces__msg__YoloTargetBias__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
bool
tb4_autonav_interfaces__msg__YoloTargetBias__init(tb4_autonav_interfaces__msg__YoloTargetBias * msg);

/// Finalize msg/YoloTargetBias message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
void
tb4_autonav_interfaces__msg__YoloTargetBias__fini(tb4_autonav_interfaces__msg__YoloTargetBias * msg);

/// Create msg/YoloTargetBias message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * tb4_autonav_interfaces__msg__YoloTargetBias__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
tb4_autonav_interfaces__msg__YoloTargetBias *
tb4_autonav_interfaces__msg__YoloTargetBias__create();

/// Destroy msg/YoloTargetBias message.
/**
 * It calls
 * tb4_autonav_interfaces__msg__YoloTargetBias__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
void
tb4_autonav_interfaces__msg__YoloTargetBias__destroy(tb4_autonav_interfaces__msg__YoloTargetBias * msg);

/// Check for msg/YoloTargetBias message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
bool
tb4_autonav_interfaces__msg__YoloTargetBias__are_equal(const tb4_autonav_interfaces__msg__YoloTargetBias * lhs, const tb4_autonav_interfaces__msg__YoloTargetBias * rhs);

/// Copy a msg/YoloTargetBias message.
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
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
bool
tb4_autonav_interfaces__msg__YoloTargetBias__copy(
  const tb4_autonav_interfaces__msg__YoloTargetBias * input,
  tb4_autonav_interfaces__msg__YoloTargetBias * output);

/// Initialize array of msg/YoloTargetBias messages.
/**
 * It allocates the memory for the number of elements and calls
 * tb4_autonav_interfaces__msg__YoloTargetBias__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
bool
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__init(tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * array, size_t size);

/// Finalize array of msg/YoloTargetBias messages.
/**
 * It calls
 * tb4_autonav_interfaces__msg__YoloTargetBias__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
void
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__fini(tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * array);

/// Create array of msg/YoloTargetBias messages.
/**
 * It allocates the memory for the array and calls
 * tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence *
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__create(size_t size);

/// Destroy array of msg/YoloTargetBias messages.
/**
 * It calls
 * tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
void
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__destroy(tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * array);

/// Check for msg/YoloTargetBias message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
bool
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__are_equal(const tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * lhs, const tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * rhs);

/// Copy an array of msg/YoloTargetBias messages.
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
ROSIDL_GENERATOR_C_PUBLIC_tb4_autonav_interfaces
bool
tb4_autonav_interfaces__msg__YoloTargetBias__Sequence__copy(
  const tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * input,
  tb4_autonav_interfaces__msg__YoloTargetBias__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__FUNCTIONS_H_
