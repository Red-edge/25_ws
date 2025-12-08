// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__STRUCT_H_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/YoloTargetBias in the package tb4_autonav_interfaces.
/**
  * tb4_autonav_interfaces/msg/YoloTargetBias.msg
 */
typedef struct tb4_autonav_interfaces__msg__YoloTargetBias
{
  std_msgs__msg__Header header;
  /// 是否当前有目标（true = 有，false = 没有）
  bool has_target;
  /// 目标类型："RED" / "GREEN" / "STOP_SIGN" / "NONE"
  rosidl_runtime_c__String type;
  /// 目标距离（米），未知时设为 -1.0
  float distance_m;
  /// 像素归一化偏差：以图像中心为 0
  /// u_norm: 水平偏移，[-1, 1]，左负右正
  /// v_norm: 垂直偏移，[-1, 1]，上负下正
  float u_norm;
  float v_norm;
} tb4_autonav_interfaces__msg__YoloTargetBias;

// Struct for a sequence of tb4_autonav_interfaces__msg__YoloTargetBias.
typedef struct tb4_autonav_interfaces__msg__YoloTargetBias__Sequence
{
  tb4_autonav_interfaces__msg__YoloTargetBias * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tb4_autonav_interfaces__msg__YoloTargetBias__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__STRUCT_H_
