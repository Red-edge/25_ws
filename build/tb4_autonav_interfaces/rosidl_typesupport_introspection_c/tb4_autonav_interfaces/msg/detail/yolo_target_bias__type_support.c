// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__rosidl_typesupport_introspection_c.h"
#include "tb4_autonav_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__functions.h"
#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `type`
#include "rosidl_runtime_c/string_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tb4_autonav_interfaces__msg__YoloTargetBias__init(message_memory);
}

void tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_fini_function(void * message_memory)
{
  tb4_autonav_interfaces__msg__YoloTargetBias__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_member_array[6] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tb4_autonav_interfaces__msg__YoloTargetBias, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "has_target",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tb4_autonav_interfaces__msg__YoloTargetBias, has_target),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "type",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_STRING,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tb4_autonav_interfaces__msg__YoloTargetBias, type),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "distance_m",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tb4_autonav_interfaces__msg__YoloTargetBias, distance_m),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "u_norm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tb4_autonav_interfaces__msg__YoloTargetBias, u_norm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "v_norm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tb4_autonav_interfaces__msg__YoloTargetBias, v_norm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_members = {
  "tb4_autonav_interfaces__msg",  // message namespace
  "YoloTargetBias",  // message name
  6,  // number of fields
  sizeof(tb4_autonav_interfaces__msg__YoloTargetBias),
  tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_member_array,  // message members
  tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_init_function,  // function to initialize message memory (memory has to be allocated)
  tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_type_support_handle = {
  0,
  &tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tb4_autonav_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tb4_autonav_interfaces, msg, YoloTargetBias)() {
  tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  if (!tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_type_support_handle.typesupport_identifier) {
    tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tb4_autonav_interfaces__msg__YoloTargetBias__rosidl_typesupport_introspection_c__YoloTargetBias_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
