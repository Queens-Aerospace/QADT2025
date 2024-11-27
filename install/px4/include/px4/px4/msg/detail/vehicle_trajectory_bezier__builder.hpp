// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/VehicleTrajectoryBezier.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__VEHICLE_TRAJECTORY_BEZIER__BUILDER_HPP_
#define PX4__MSG__DETAIL__VEHICLE_TRAJECTORY_BEZIER__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/vehicle_trajectory_bezier__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_VehicleTrajectoryBezier_bezier_order
{
public:
  explicit Init_VehicleTrajectoryBezier_bezier_order(::px4::msg::VehicleTrajectoryBezier & msg)
  : msg_(msg)
  {}
  ::px4::msg::VehicleTrajectoryBezier bezier_order(::px4::msg::VehicleTrajectoryBezier::_bezier_order_type arg)
  {
    msg_.bezier_order = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::VehicleTrajectoryBezier msg_;
};

class Init_VehicleTrajectoryBezier_control_points
{
public:
  explicit Init_VehicleTrajectoryBezier_control_points(::px4::msg::VehicleTrajectoryBezier & msg)
  : msg_(msg)
  {}
  Init_VehicleTrajectoryBezier_bezier_order control_points(::px4::msg::VehicleTrajectoryBezier::_control_points_type arg)
  {
    msg_.control_points = std::move(arg);
    return Init_VehicleTrajectoryBezier_bezier_order(msg_);
  }

private:
  ::px4::msg::VehicleTrajectoryBezier msg_;
};

class Init_VehicleTrajectoryBezier_timestamp
{
public:
  Init_VehicleTrajectoryBezier_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_VehicleTrajectoryBezier_control_points timestamp(::px4::msg::VehicleTrajectoryBezier::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_VehicleTrajectoryBezier_control_points(msg_);
  }

private:
  ::px4::msg::VehicleTrajectoryBezier msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::VehicleTrajectoryBezier>()
{
  return px4::msg::builder::Init_VehicleTrajectoryBezier_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__VEHICLE_TRAJECTORY_BEZIER__BUILDER_HPP_
