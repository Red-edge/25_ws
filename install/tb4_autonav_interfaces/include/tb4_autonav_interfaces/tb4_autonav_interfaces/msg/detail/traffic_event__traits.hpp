// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tb4_autonav_interfaces:msg/TrafficEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__TRAITS_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tb4_autonav_interfaces/msg/detail/traffic_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace tb4_autonav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const TrafficEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: is_ready
  {
    out << "is_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.is_ready, out);
    out << ", ";
  }

  // member: type
  {
    out << "type: ";
    rosidl_generator_traits::value_to_yaml(msg.type, out);
    out << ", ";
  }

  // member: distance
  {
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TrafficEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: is_ready
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "is_ready: ";
    rosidl_generator_traits::value_to_yaml(msg.is_ready, out);
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

  // member: distance
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "distance: ";
    rosidl_generator_traits::value_to_yaml(msg.distance, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TrafficEvent & msg, bool use_flow_style = false)
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
  const tb4_autonav_interfaces::msg::TrafficEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  tb4_autonav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tb4_autonav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tb4_autonav_interfaces::msg::TrafficEvent & msg)
{
  return tb4_autonav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tb4_autonav_interfaces::msg::TrafficEvent>()
{
  return "tb4_autonav_interfaces::msg::TrafficEvent";
}

template<>
inline const char * name<tb4_autonav_interfaces::msg::TrafficEvent>()
{
  return "tb4_autonav_interfaces/msg/TrafficEvent";
}

template<>
struct has_fixed_size<tb4_autonav_interfaces::msg::TrafficEvent>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<tb4_autonav_interfaces::msg::TrafficEvent>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<tb4_autonav_interfaces::msg::TrafficEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__TRAITS_HPP_
