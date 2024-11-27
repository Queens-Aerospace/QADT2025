// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/OrbTestMedium.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ORB_TEST_MEDIUM__BUILDER_HPP_
#define PX4__MSG__DETAIL__ORB_TEST_MEDIUM__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/orb_test_medium__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_OrbTestMedium_junk
{
public:
  explicit Init_OrbTestMedium_junk(::px4::msg::OrbTestMedium & msg)
  : msg_(msg)
  {}
  ::px4::msg::OrbTestMedium junk(::px4::msg::OrbTestMedium::_junk_type arg)
  {
    msg_.junk = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::OrbTestMedium msg_;
};

class Init_OrbTestMedium_val
{
public:
  explicit Init_OrbTestMedium_val(::px4::msg::OrbTestMedium & msg)
  : msg_(msg)
  {}
  Init_OrbTestMedium_junk val(::px4::msg::OrbTestMedium::_val_type arg)
  {
    msg_.val = std::move(arg);
    return Init_OrbTestMedium_junk(msg_);
  }

private:
  ::px4::msg::OrbTestMedium msg_;
};

class Init_OrbTestMedium_timestamp
{
public:
  Init_OrbTestMedium_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrbTestMedium_val timestamp(::px4::msg::OrbTestMedium::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_OrbTestMedium_val(msg_);
  }

private:
  ::px4::msg::OrbTestMedium msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::OrbTestMedium>()
{
  return px4::msg::builder::Init_OrbTestMedium_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ORB_TEST_MEDIUM__BUILDER_HPP_
