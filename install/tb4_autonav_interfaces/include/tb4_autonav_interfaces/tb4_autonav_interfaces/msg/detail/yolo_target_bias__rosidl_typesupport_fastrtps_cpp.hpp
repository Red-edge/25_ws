// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "tb4_autonav_interfaces/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace tb4_autonav_interfaces
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tb4_autonav_interfaces
cdr_serialize(
  const tb4_autonav_interfaces::msg::YoloTargetBias & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tb4_autonav_interfaces
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  tb4_autonav_interfaces::msg::YoloTargetBias & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tb4_autonav_interfaces
get_serialized_size(
  const tb4_autonav_interfaces::msg::YoloTargetBias & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tb4_autonav_interfaces
max_serialized_size_YoloTargetBias(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace tb4_autonav_interfaces

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_tb4_autonav_interfaces
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, tb4_autonav_interfaces, msg, YoloTargetBias)();

#ifdef __cplusplus
}
#endif

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
