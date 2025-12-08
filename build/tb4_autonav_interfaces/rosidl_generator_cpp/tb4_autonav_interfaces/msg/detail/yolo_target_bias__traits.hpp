// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__TRAITS_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace tb4_autonav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const YoloTargetBias & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: has_target
  {
    out << "has_target: ";
    rosidl_generator_traits::value_to_yaml(msg.has_target, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: distance_m
  {
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
    out << ", ";
  }

  // member: u_norm
  {
    out << "u_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.u_norm, out);
    out << ", ";
  }

  // member: v_norm
  {
    out << "v_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.v_norm, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const YoloTargetBias & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: has_target
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "has_target: ";
    rosidl_generator_traits::value_to_yaml(msg.has_target, out);
    out << "\n";
  }

  // member: type
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << "\n";
  }

  // member: distance_m
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance_m: ";
    rosidl_generator_traits::value_to_yaml(msg.distance_m, out);
    out << "\n";
  }

  // member: u_norm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "u_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.u_norm, out);
    out << "\n";
  }

  // member: v_norm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "v_norm: ";
    rosidl_generator_traits::value_to_yaml(msg.v_norm, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const YoloTargetBias & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace tb4_autonav_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use tb4_autonav_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const tb4_autonav_interfaces::msg::YoloTargetBias & msg,
  std::ostream & out, size_t indentation = 0)
{
  tb4_autonav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tb4_autonav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tb4_autonav_interfaces::msg::YoloTargetBias & msg)
{
  return tb4_autonav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tb4_autonav_interfaces::msg::YoloTargetBias>()
{
  return "tb4_autonav_interfaces::msg::YoloTargetBias";
}

template<>
inline const char * name<tb4_autonav_interfaces::msg::YoloTargetBias>()
{
  return "tb4_autonav_interfaces/msg/YoloTargetBias";
}

template<>
struct has_fixed_size<tb4_autonav_interfaces::msg::YoloTargetBias>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tb4_autonav_interfaces::msg::YoloTargetBias>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tb4_autonav_interfaces::msg::YoloTargetBias>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__TRAITS_HPP_
