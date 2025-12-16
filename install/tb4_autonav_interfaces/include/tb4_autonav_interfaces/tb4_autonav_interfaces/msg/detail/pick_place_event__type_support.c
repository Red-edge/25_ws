// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from tb4_autonav_interfaces:msg/PickPlaceEvent.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "tb4_autonav_interfaces/msg/detail/pick_place_event__rosidl_typesupport_introspection_c.h"
#include "tb4_autonav_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "tb4_autonav_interfaces/msg/detail/pick_place_event__functions.h"
#include "tb4_autonav_interfaces/msg/detail/pick_place_event__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  tb4_autonav_interfaces__msg__PickPlaceEvent__init(message_memory);
}

void tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_fini_function(void * message_memory)
{
  tb4_autonav_interfaces__msg__PickPlaceEvent__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_member_array[1] = {
  {
    "status",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(tb4_autonav_interfaces__msg__PickPlaceEvent, status),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_members = {
  "tb4_autonav_interfaces__msg",  // message namespace
  "PickPlaceEvent",  // message name
  1,  // number of fields
  sizeof(tb4_autonav_interfaces__msg__PickPlaceEvent),
  tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_member_array,  // message members
  tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_init_function,  // function to initialize message memory (memory has to be allocated)
  tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_type_support_handle = {
  0,
  &tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_tb4_autonav_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, tb4_autonav_interfaces, msg, PickPlaceEvent)() {
  if (!tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_type_support_handle.typesupport_identifier) {
    tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &tb4_autonav_interfaces__msg__PickPlaceEvent__rosidl_typesupport_introspection_c__PickPlaceEvent_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
