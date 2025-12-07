// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from demo_interfaces_test:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__TRAITS_HPP_
#define DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "demo_interfaces_test/msg/detail/test_msg__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace demo_interfaces_test
{

namespace msg
{

inline void to_flow_style_yaml(
  const TestMsg & msg,
  std::ostream & out)
{
  out << "{";
  // member: data
  {
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const TestMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "data: ";
    rosidl_generator_traits::value_to_yaml(msg.data, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const TestMsg & msg, bool use_flow_style = false)
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

}  // namespace demo_interfaces_test

namespace rosidl_generator_traits
{

[[deprecated("use demo_interfaces_test::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const demo_interfaces_test::msg::TestMsg & msg,
  std::ostream & out, size_t indentation = 0)
{
  demo_interfaces_test::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use demo_interfaces_test::msg::to_yaml() instead")]]
inline std::string to_yaml(const demo_interfaces_test::msg::TestMsg & msg)
{
  return demo_interfaces_test::msg::to_yaml(msg);
}

template<>
inline const char * data_type<demo_interfaces_test::msg::TestMsg>()
{
  return "demo_interfaces_test::msg::TestMsg";
}

template<>
inline const char * name<demo_interfaces_test::msg::TestMsg>()
{
  return "demo_interfaces_test/msg/TestMsg";
}

template<>
struct has_fixed_size<demo_interfaces_test::msg::TestMsg>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<demo_interfaces_test::msg::TestMsg>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<demo_interfaces_test::msg::TestMsg>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__TRAITS_HPP_
