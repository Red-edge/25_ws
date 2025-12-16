// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tb4_autonav_interfaces:msg/PickPlaceEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__STRUCT_H_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/PickPlaceEvent in the package tb4_autonav_interfaces.
/**
  * 状态标志 status%2 = 0 Nav开
  * 0: 未到达  Nav开
  * 1: 到达 Nav关 Pick开
  * 2: Pick结束， Nav开 Pick关
  * 3: 放置到达， Nav关 Place开
  * 4：放置完毕 Nav开
 */
typedef struct tb4_autonav_interfaces__msg__PickPlaceEvent
{
  int32_t status;
} tb4_autonav_interfaces__msg__PickPlaceEvent;

// Struct for a sequence of tb4_autonav_interfaces__msg__PickPlaceEvent.
typedef struct tb4_autonav_interfaces__msg__PickPlaceEvent__Sequence
{
  tb4_autonav_interfaces__msg__PickPlaceEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tb4_autonav_interfaces__msg__PickPlaceEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__STRUCT_H_
