// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/WorldControl.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'reset'
#include "ros_gz_interfaces/msg/detail/world_reset__struct.hpp"
// Member 'run_to_sim_time'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__WorldControl __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__WorldControl __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct WorldControl_
{
  using Type = WorldControl_<ContainerAllocator>;

  explicit WorldControl_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reset(_init),
    run_to_sim_time(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->multi_step = 0ul;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->pause = false;
      this->step = false;
      this->multi_step = 0ul;
      this->seed = 0ul;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pause = false;
      this->step = false;
      this->seed = 0ul;
    }
  }

  explicit WorldControl_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : reset(_alloc, _init),
    run_to_sim_time(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::DEFAULTS_ONLY == _init)
    {
      this->multi_step = 0ul;
    } else if (rosidl_runtime_cpp::MessageInitialization::ZERO == _init) {
      this->pause = false;
      this->step = false;
      this->multi_step = 0ul;
      this->seed = 0ul;
    }
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->pause = false;
      this->step = false;
      this->seed = 0ul;
    }
  }

  // field types and members
  using _pause_type =
    bool;
  _pause_type pause;
  using _step_type =
    bool;
  _step_type step;
  using _multi_step_type =
    uint32_t;
  _multi_step_type multi_step;
  using _reset_type =
    ros_gz_interfaces::msg::WorldReset_<ContainerAllocator>;
  _reset_type reset;
  using _seed_type =
    uint32_t;
  _seed_type seed;
  using _run_to_sim_time_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _run_to_sim_time_type run_to_sim_time;

  // setters for named parameter idiom
  Type & set__pause(
    const bool & _arg)
  {
    this->pause = _arg;
    return *this;
  }
  Type & set__step(
    const bool & _arg)
  {
    this->step = _arg;
    return *this;
  }
  Type & set__multi_step(
    const uint32_t & _arg)
  {
    this->multi_step = _arg;
    return *this;
  }
  Type & set__reset(
    const ros_gz_interfaces::msg::WorldReset_<ContainerAllocator> & _arg)
  {
    this->reset = _arg;
    return *this;
  }
  Type & set__seed(
    const uint32_t & _arg)
  {
    this->seed = _arg;
    return *this;
  }
  Type & set__run_to_sim_time(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->run_to_sim_time = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::WorldControl_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::WorldControl_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::WorldControl_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::WorldControl_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__WorldControl
    std::shared_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__WorldControl
    std::shared_ptr<ros_gz_interfaces::msg::WorldControl_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const WorldControl_ & other) const
  {
    if (this->pause != other.pause) {
      return false;
    }
    if (this->step != other.step) {
      return false;
    }
    if (this->multi_step != other.multi_step) {
      return false;
    }
    if (this->reset != other.reset) {
      return false;
    }
    if (this->seed != other.seed) {
      return false;
    }
    if (this->run_to_sim_time != other.run_to_sim_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const WorldControl_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct WorldControl_

// alias to use template instance with default allocator
using WorldControl =
  ros_gz_interfaces::msg::WorldControl_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__WORLD_CONTROL__STRUCT_HPP_
