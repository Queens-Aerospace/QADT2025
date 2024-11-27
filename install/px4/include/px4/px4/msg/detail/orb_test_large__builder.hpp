// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from px4:msg/OrbTestLarge.idl
// generated code does not contain a copyright notice

#ifndef PX4__MSG__DETAIL__ORB_TEST_LARGE__BUILDER_HPP_
#define PX4__MSG__DETAIL__ORB_TEST_LARGE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "px4/msg/detail/orb_test_large__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace px4
{

namespace msg
{

namespace builder
{

class Init_OrbTestLarge_junk
{
public:
  explicit Init_OrbTestLarge_junk(::px4::msg::OrbTestLarge & msg)
  : msg_(msg)
  {}
  ::px4::msg::OrbTestLarge junk(::px4::msg::OrbTestLarge::_junk_type arg)
  {
    msg_.junk = std::move(arg);
    return std::move(msg_);
  }

private:
  ::px4::msg::OrbTestLarge msg_;
};

class Init_OrbTestLarge_val
{
public:
  explicit Init_OrbTestLarge_val(::px4::msg::OrbTestLarge & msg)
  : msg_(msg)
  {}
  Init_OrbTestLarge_junk val(::px4::msg::OrbTestLarge::_val_type arg)
  {
    msg_.val = std::move(arg);
    return Init_OrbTestLarge_junk(msg_);
  }

private:
  ::px4::msg::OrbTestLarge msg_;
};

class Init_OrbTestLarge_timestamp
{
public:
  Init_OrbTestLarge_timestamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_OrbTestLarge_val timestamp(::px4::msg::OrbTestLarge::_timestamp_type arg)
  {
    msg_.timestamp = std::move(arg);
    return Init_OrbTestLarge_val(msg_);
  }

private:
  ::px4::msg::OrbTestLarge msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::px4::msg::OrbTestLarge>()
{
  return px4::msg::builder::Init_OrbTestLarge_timestamp();
}

}  // namespace px4

#endif  // PX4__MSG__DETAIL__ORB_TEST_LARGE__BUILDER_HPP_
