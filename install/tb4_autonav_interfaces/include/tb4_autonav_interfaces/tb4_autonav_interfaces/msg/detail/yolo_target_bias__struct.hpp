// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__STRUCT_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__tb4_autonav_interfaces__msg__YoloTargetBias __attribute__((deprecated))
#else
# define DEPRECATED__tb4_autonav_interfaces__msg__YoloTargetBias __declspec(deprecated)
#endif

namespace tb4_autonav_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct YoloTargetBias_
{
  using Type = YoloTargetBias_<ContainerAllocator>;

  explicit YoloTargetBias_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->has_target = false;
      this->type = "";
      this->distance_m = 0.0f;
      this->u_norm = 0.0f;
      this->v_norm = 0.0f;
    }
  }

  explicit YoloTargetBias_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    type(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->has_target = false;
      this->type = "";
      this->distance_m = 0.0f;
      this->u_norm = 0.0f;
      this->v_norm = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _has_target_type =
    bool;
  _has_target_type has_target;
  using _type_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _type_type type;
  using _distance_m_type =
    float;
  _distance_m_type distance_m;
  using _u_norm_type =
    float;
  _u_norm_type u_norm;
  using _v_norm_type =
    float;
  _v_norm_type v_norm;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__has_target(
    const bool & _arg)
  {
    this->has_target = _arg;
    return *this;
  }
  Type & set__type(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__distance_m(
    const float & _arg)
  {
    this->distance_m = _arg;
    return *this;
  }
  Type & set__u_norm(
    const float & _arg)
  {
    this->u_norm = _arg;
    return *this;
  }
  Type & set__v_norm(
    const float & _arg)
  {
    this->v_norm = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator> *;
  using ConstRawPtr =
    const tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__tb4_autonav_interfaces__msg__YoloTargetBias
    std::shared_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__tb4_autonav_interfaces__msg__YoloTargetBias
    std::shared_ptr<tb4_autonav_interfaces::msg::YoloTargetBias_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const YoloTargetBias_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->has_target != other.has_target) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->distance_m != other.distance_m) {
      return false;
    }
    if (this->u_norm != other.u_norm) {
      return false;
    }
    if (this->v_norm != other.v_norm) {
      return false;
    }
    return true;
  }
  bool operator!=(const YoloTargetBias_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct YoloTargetBias_

// alias to use template instance with default allocator
using YoloTargetBias =
  tb4_autonav_interfaces::msg::YoloTargetBias_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace tb4_autonav_interfaces

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__STRUCT_HPP_
