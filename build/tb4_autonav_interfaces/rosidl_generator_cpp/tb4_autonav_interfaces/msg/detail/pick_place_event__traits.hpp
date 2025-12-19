// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from tb4_autonav_interfaces:msg/PickPlaceEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__TRAITS_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "tb4_autonav_interfaces/msg/detail/pick_place_event__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace tb4_autonav_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const PickPlaceEvent & msg,
  std::ostream & out)
{
  out << "{";
  // member: status
  {
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PickPlaceEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "status: ";
    rosidl_generator_traits::value_to_yaml(msg.status, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PickPlaceEvent & msg, bool use_flow_style = false)
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
  const tb4_autonav_interfaces::msg::PickPlaceEvent & msg,
  std::ostream & out, size_t indentation = 0)
{
  tb4_autonav_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use tb4_autonav_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const tb4_autonav_interfaces::msg::PickPlaceEvent & msg)
{
  return tb4_autonav_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<tb4_autonav_interfaces::msg::PickPlaceEvent>()
{
  return "tb4_autonav_interfaces::msg::PickPlaceEvent";
}

template<>
inline const char * name<tb4_autonav_interfaces::msg::PickPlaceEvent>()
{
  return "tb4_autonav_interfaces/msg/PickPlaceEvent";
}

template<>
struct has_fixed_size<tb4_autonav_interfaces::msg::PickPlaceEvent>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<tb4_autonav_interfaces::msg::PickPlaceEvent>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<tb4_autonav_interfaces::msg::PickPlaceEvent>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__TRAITS_HPP_
