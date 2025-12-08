// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tb4_autonav_interfaces:msg/YoloTargetBias.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__BUILDER_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tb4_autonav_interfaces/msg/detail/yolo_target_bias__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tb4_autonav_interfaces
{

namespace msg
{

namespace builder
{

class Init_YoloTargetBias_v_norm
{
public:
  explicit Init_YoloTargetBias_v_norm(::tb4_autonav_interfaces::msg::YoloTargetBias & msg)
  : msg_(msg)
  {}
  ::tb4_autonav_interfaces::msg::YoloTargetBias v_norm(::tb4_autonav_interfaces::msg::YoloTargetBias::_v_norm_type arg)
  {
    msg_.v_norm = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::YoloTargetBias msg_;
};

class Init_YoloTargetBias_u_norm
{
public:
  explicit Init_YoloTargetBias_u_norm(::tb4_autonav_interfaces::msg::YoloTargetBias & msg)
  : msg_(msg)
  {}
  Init_YoloTargetBias_v_norm u_norm(::tb4_autonav_interfaces::msg::YoloTargetBias::_u_norm_type arg)
  {
    msg_.u_norm = std::move(arg);
    return Init_YoloTargetBias_v_norm(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::YoloTargetBias msg_;
};

class Init_YoloTargetBias_distance_m
{
public:
  explicit Init_YoloTargetBias_distance_m(::tb4_autonav_interfaces::msg::YoloTargetBias & msg)
  : msg_(msg)
  {}
  Init_YoloTargetBias_u_norm distance_m(::tb4_autonav_interfaces::msg::YoloTargetBias::_distance_m_type arg)
  {
    msg_.distance_m = std::move(arg);
    return Init_YoloTargetBias_u_norm(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::YoloTargetBias msg_;
};

class Init_YoloTargetBias_type
{
public:
  explicit Init_YoloTargetBias_type(::tb4_autonav_interfaces::msg::YoloTargetBias & msg)
  : msg_(msg)
  {}
  Init_YoloTargetBias_distance_m type(::tb4_autonav_interfaces::msg::YoloTargetBias::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_YoloTargetBias_distance_m(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::YoloTargetBias msg_;
};

class Init_YoloTargetBias_has_target
{
public:
  explicit Init_YoloTargetBias_has_target(::tb4_autonav_interfaces::msg::YoloTargetBias & msg)
  : msg_(msg)
  {}
  Init_YoloTargetBias_type has_target(::tb4_autonav_interfaces::msg::YoloTargetBias::_has_target_type arg)
  {
    msg_.has_target = std::move(arg);
    return Init_YoloTargetBias_type(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::YoloTargetBias msg_;
};

class Init_YoloTargetBias_header
{
public:
  Init_YoloTargetBias_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_YoloTargetBias_has_target header(::tb4_autonav_interfaces::msg::YoloTargetBias::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_YoloTargetBias_has_target(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::YoloTargetBias msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tb4_autonav_interfaces::msg::YoloTargetBias>()
{
  return tb4_autonav_interfaces::msg::builder::Init_YoloTargetBias_header();
}

}  // namespace tb4_autonav_interfaces

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__YOLO_TARGET_BIAS__BUILDER_HPP_
