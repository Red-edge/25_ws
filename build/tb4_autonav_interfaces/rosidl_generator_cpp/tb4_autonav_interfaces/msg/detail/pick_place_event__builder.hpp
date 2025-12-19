// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tb4_autonav_interfaces:msg/PickPlaceEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__BUILDER_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tb4_autonav_interfaces/msg/detail/pick_place_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tb4_autonav_interfaces
{

namespace msg
{

namespace builder
{

class Init_PickPlaceEvent_status
{
public:
  Init_PickPlaceEvent_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::tb4_autonav_interfaces::msg::PickPlaceEvent status(::tb4_autonav_interfaces::msg::PickPlaceEvent::_status_type arg)
  {
    msg_.status = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::PickPlaceEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tb4_autonav_interfaces::msg::PickPlaceEvent>()
{
  return tb4_autonav_interfaces::msg::builder::Init_PickPlaceEvent_status();
}

}  // namespace tb4_autonav_interfaces

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__PICK_PLACE_EVENT__BUILDER_HPP_
