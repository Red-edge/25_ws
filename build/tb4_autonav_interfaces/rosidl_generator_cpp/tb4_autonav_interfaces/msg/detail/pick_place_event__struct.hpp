// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tb4_autonav_interfaces:msg/PickPlaceEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__STRUCT_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__tb4_autonav_interfaces__msg__PickPlaceEvent __attribute__((deprecated))
#else
# define DEPRECATED__tb4_autonav_interfaces__msg__PickPlaceEvent __declspec(deprecated)
#endif

namespace tb4_autonav_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PickPlaceEvent_
{
  using Type = PickPlaceEvent_<ContainerAllocator>;

  explicit PickPlaceEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0l;
    }
  }

  explicit PickPlaceEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->status = 0l;
    }
  }

  // field types and members
  using _status_type =
    int32_t;
  _status_type status;

  // setters for named parameter idiom
  Type & set__status(
    const int32_t & _arg)
  {
    this->status = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tb4_autonav_interfaces__msg__PickPlaceEvent
    std::shared_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tb4_autonav_interfaces__msg__PickPlaceEvent
    std::shared_ptr<tb4_autonav_interfaces::msg::PickPlaceEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PickPlaceEvent_ & other) const
  {
    if (this->status != other.status) {
      return false;
    }
    return true;
  }
  bool operator!=(const PickPlaceEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PickPlaceEvent_

// alias to use template instance with default allocator
using PickPlaceEvent =
  tb4_autonav_interfaces::msg::PickPlaceEvent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tb4_autonav_interfaces

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__STRUCT_HPP_
