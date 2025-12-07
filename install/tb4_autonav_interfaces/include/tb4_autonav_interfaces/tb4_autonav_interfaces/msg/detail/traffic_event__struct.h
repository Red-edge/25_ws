// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from tb4_autonav_interfaces:msg/TrafficEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__STRUCT_H_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"
// Member 'type'
#include "rosidl_runtime_c/string.h"

/// Struct defined in msg/TrafficEvent in the package tb4_autonav_interfaces.
/**
  * TrafficEvent.msg
  * 视觉节点和导航节点之间传递“视觉是否就绪 + 交通事件检测结果”
 */
typedef struct tb4_autonav_interfaces__msg__TrafficEvent
{
  builtin_interfaces__msg__Time stamp;
  /// 视觉节点是否已经就绪：
  ///   false：节点刚启动，还没收到相机图像
  ///   true ：至少成功处理过一帧图像
  bool is_ready;
  /// 检测到的事件类型：
  ///   "RED"        红灯
  ///   "GREEN"      绿灯
  ///   "STOP_SIGN"  停止标志
  ///   "NONE"       无关键事件
  rosidl_runtime_c__String type;
  /// 事件的粗略距离，单位 m；若不可估计则为 -1.0
  float distance;
} tb4_autonav_interfaces__msg__TrafficEvent;

// Struct for a sequence of tb4_autonav_interfaces__msg__TrafficEvent.
typedef struct tb4_autonav_interfaces__msg__TrafficEvent__Sequence
{
  tb4_autonav_interfaces__msg__TrafficEvent * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} tb4_autonav_interfaces__msg__TrafficEvent__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__STRUCT_H_
