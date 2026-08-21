// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from race_msgs:msg/FlightAltitudeReference.idl
// generated code does not contain a copyright notice

#ifndef RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__BUILDER_HPP_
#define RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "race_msgs/msg/detail/flight_altitude_reference__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace race_msgs
{

namespace msg
{

namespace builder
{

class Init_FlightAltitudeReference_target_z_map
{
public:
  explicit Init_FlightAltitudeReference_target_z_map(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  ::race_msgs::msg::FlightAltitudeReference target_z_map(::race_msgs::msg::FlightAltitudeReference::_target_z_map_type arg)
  {
    msg_.target_z_map = std::move(arg);
    return std::move(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_ground_z_map
{
public:
  explicit Init_FlightAltitudeReference_ground_z_map(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_target_z_map ground_z_map(::race_msgs::msg::FlightAltitudeReference::_ground_z_map_type arg)
  {
    msg_.ground_z_map = std::move(arg);
    return Init_FlightAltitudeReference_target_z_map(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_target_z_local_ned
{
public:
  explicit Init_FlightAltitudeReference_target_z_local_ned(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_ground_z_map target_z_local_ned(::race_msgs::msg::FlightAltitudeReference::_target_z_local_ned_type arg)
  {
    msg_.target_z_local_ned = std::move(arg);
    return Init_FlightAltitudeReference_ground_z_map(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_ground_z_local_ned
{
public:
  explicit Init_FlightAltitudeReference_ground_z_local_ned(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_target_z_local_ned ground_z_local_ned(::race_msgs::msg::FlightAltitudeReference::_ground_z_local_ned_type arg)
  {
    msg_.ground_z_local_ned = std::move(arg);
    return Init_FlightAltitudeReference_target_z_local_ned(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_max_agl_m
{
public:
  explicit Init_FlightAltitudeReference_max_agl_m(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_ground_z_local_ned max_agl_m(::race_msgs::msg::FlightAltitudeReference::_max_agl_m_type arg)
  {
    msg_.max_agl_m = std::move(arg);
    return Init_FlightAltitudeReference_ground_z_local_ned(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_min_agl_m
{
public:
  explicit Init_FlightAltitudeReference_min_agl_m(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_max_agl_m min_agl_m(::race_msgs::msg::FlightAltitudeReference::_min_agl_m_type arg)
  {
    msg_.min_agl_m = std::move(arg);
    return Init_FlightAltitudeReference_max_agl_m(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_target_agl_m
{
public:
  explicit Init_FlightAltitudeReference_target_agl_m(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_min_agl_m target_agl_m(::race_msgs::msg::FlightAltitudeReference::_target_agl_m_type arg)
  {
    msg_.target_agl_m = std::move(arg);
    return Init_FlightAltitudeReference_min_agl_m(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_valid
{
public:
  explicit Init_FlightAltitudeReference_valid(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_target_agl_m valid(::race_msgs::msg::FlightAltitudeReference::_valid_type arg)
  {
    msg_.valid = std::move(arg);
    return Init_FlightAltitudeReference_target_agl_m(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_flight_id
{
public:
  explicit Init_FlightAltitudeReference_flight_id(::race_msgs::msg::FlightAltitudeReference & msg)
  : msg_(msg)
  {}
  Init_FlightAltitudeReference_valid flight_id(::race_msgs::msg::FlightAltitudeReference::_flight_id_type arg)
  {
    msg_.flight_id = std::move(arg);
    return Init_FlightAltitudeReference_valid(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

class Init_FlightAltitudeReference_header
{
public:
  Init_FlightAltitudeReference_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FlightAltitudeReference_flight_id header(::race_msgs::msg::FlightAltitudeReference::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_FlightAltitudeReference_flight_id(msg_);
  }

private:
  ::race_msgs::msg::FlightAltitudeReference msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::race_msgs::msg::FlightAltitudeReference>()
{
  return race_msgs::msg::builder::Init_FlightAltitudeReference_header();
}

}  // namespace race_msgs

#endif  // RACE_MSGS__MSG__DETAIL__FLIGHT_ALTITUDE_REFERENCE__BUILDER_HPP_
