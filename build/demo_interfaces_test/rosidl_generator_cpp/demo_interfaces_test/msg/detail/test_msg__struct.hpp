// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from demo_interfaces_test:msg/TestMsg.idl
// generated code does not contain a copyright notice

#ifndef DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__STRUCT_HPP_
#define DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__demo_interfaces_test__msg__TestMsg __attribute__((deprecated))
#else
# define DEPRECATED__demo_interfaces_test__msg__TestMsg __declspec(deprecated)
#endif

namespace demo_interfaces_test
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TestMsg_
{
  using Type = TestMsg_<ContainerAllocator>;

  explicit TestMsg_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  explicit TestMsg_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : data(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->data = "";
    }
  }

  // field types and members
  using _data_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _data_type data;

  // setters for named parameter idiom
  Type & set__data(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->data = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    demo_interfaces_test::msg::TestMsg_<ContainerAllocator> *;
  using ConstRawPtr =
    const demo_interfaces_test::msg::TestMsg_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      demo_interfaces_test::msg::TestMsg_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      demo_interfaces_test::msg::TestMsg_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__demo_interfaces_test__msg__TestMsg
    std::shared_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__demo_interfaces_test__msg__TestMsg
    std::shared_ptr<demo_interfaces_test::msg::TestMsg_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TestMsg_ & other) const
  {
    if (this->data != other.data) {
      return false;
    }
    return true;
  }
  bool operator!=(const TestMsg_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TestMsg_

// alias to use template instance with default allocator
using TestMsg =
  demo_interfaces_test::msg::TestMsg_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace demo_interfaces_test

#endif  // DEMO_INTERFACES_TEST__MSG__DETAIL__TEST_MSG__STRUCT_HPP_
