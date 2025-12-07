// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tb4_autonav_interfaces:msg/TrafficEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__STRUCT_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tb4_autonav_interfaces__msg__TrafficEvent __attribute__((deprecated))
#else
# define DEPRECATED__tb4_autonav_interfaces__msg__TrafficEvent __declspec(deprecated)
#endif

namespace tb4_autonav_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct TrafficEvent_
{
  using Type = TrafficEvent_<ContainerAllocator>;

  explicit TrafficEvent_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_ready = false;
      this->type = "";
      this->distance = 0.0f;
    }
  }

  explicit TrafficEvent_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->is_ready = false;
      this->type = "";
      this->distance = 0.0f;
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _is_ready_type =
    bool;
  _is_ready_type is_ready;
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_type type;
  using _distance_type =
    float;
  _distance_type distance;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__is_ready(
    const bool & _arg)
  {
    this->is_ready = _arg;
    return *this;
  }
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__distance(
    const float & _arg)
  {
    this->distance = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator> *;
  using ConstRawPtr =
    const tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tb4_autonav_interfaces__msg__TrafficEvent
    std::shared_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tb4_autonav_interfaces__msg__TrafficEvent
    std::shared_ptr<tb4_autonav_interfaces::msg::TrafficEvent_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const TrafficEvent_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->is_ready != other.is_ready) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->distance != other.distance) {
      return false;
    }
    return true;
  }
  bool operator!=(const TrafficEvent_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct TrafficEvent_

// alias to use template instance with default allocator
using TrafficEvent =
  tb4_autonav_interfaces::msg::TrafficEvent_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tb4_autonav_interfaces

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__STRUCT_HPP_
