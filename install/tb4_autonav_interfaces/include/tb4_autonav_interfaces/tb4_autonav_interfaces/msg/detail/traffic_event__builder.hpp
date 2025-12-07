// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from tb4_autonav_interfaces:msg/TrafficEvent.idl
// generated code does not contain a copyright notice

#ifndef TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__BUILDER_HPP_
#define TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "tb4_autonav_interfaces/msg/detail/traffic_event__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace tb4_autonav_interfaces
{

namespace msg
{

namespace builder
{

class Init_TrafficEvent_distance
{
public:
  explicit Init_TrafficEvent_distance(::tb4_autonav_interfaces::msg::TrafficEvent & msg)
  : msg_(msg)
  {}
  ::tb4_autonav_interfaces::msg::TrafficEvent distance(::tb4_autonav_interfaces::msg::TrafficEvent::_distance_type arg)
  {
    msg_.distance = std::move(arg);
    return std::move(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::TrafficEvent msg_;
};

class Init_TrafficEvent_type
{
public:
  explicit Init_TrafficEvent_type(::tb4_autonav_interfaces::msg::TrafficEvent & msg)
  : msg_(msg)
  {}
  Init_TrafficEvent_distance type(::tb4_autonav_interfaces::msg::TrafficEvent::_type_type arg)
  {
    msg_.type = std::move(arg);
    return Init_TrafficEvent_distance(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::TrafficEvent msg_;
};

class Init_TrafficEvent_is_ready
{
public:
  explicit Init_TrafficEvent_is_ready(::tb4_autonav_interfaces::msg::TrafficEvent & msg)
  : msg_(msg)
  {}
  Init_TrafficEvent_type is_ready(::tb4_autonav_interfaces::msg::TrafficEvent::_is_ready_type arg)
  {
    msg_.is_ready = std::move(arg);
    return Init_TrafficEvent_type(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::TrafficEvent msg_;
};

class Init_TrafficEvent_stamp
{
public:
  Init_TrafficEvent_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_TrafficEvent_is_ready stamp(::tb4_autonav_interfaces::msg::TrafficEvent::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_TrafficEvent_is_ready(msg_);
  }

private:
  ::tb4_autonav_interfaces::msg::TrafficEvent msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::tb4_autonav_interfaces::msg::TrafficEvent>()
{
  return tb4_autonav_interfaces::msg::builder::Init_TrafficEvent_stamp();
}

}  // namespace tb4_autonav_interfaces

#endif  // TB4_AUTONAV_INTERFACES__MSG__DETAIL__TRAFFIC_EVENT__BUILDER_HPP_
