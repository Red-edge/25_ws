// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from demo_interfaces_test:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__BUILDER_HPP_
#define DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "demo_interfaces_test/msg/detail/test_msg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace demo_interfaces_test
{

namespace msg
{

namespace builder
{

class Init_TestMsg_data
{
public:
  Init_TestMsg_data()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::demo_interfaces_test::msg::TestMsg data(::demo_interfaces_test::msg::TestMsg::_data_type arg)
  {
    msg_.data = std::move(arg);
    return std::move(msg_);
  }

private:
  ::demo_interfaces_test::msg::TestMsg msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::demo_interfaces_test::msg::TestMsg>()
{
  return demo_interfaces_test::msg::builder::Init_TestMsg_data();
}

}  // namespace demo_interfaces_test

#endif  // DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__BUILDER_HPP_
