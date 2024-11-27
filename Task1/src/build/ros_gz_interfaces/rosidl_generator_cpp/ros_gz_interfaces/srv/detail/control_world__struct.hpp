// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:srv/ControlWorld.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__SRV__DETAIL__CONTROL_WORLD__STRUCT_HPP_
#define ROS_GZ_INTERFACES__SRV__DETAIL__CONTROL_WORLD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'world_control'
#include "ros_gz_interfaces/msg/detail/world_control__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Request __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlWorld_Request_
{
  using Type = ControlWorld_Request_<ContainerAllocator>;

  explicit ControlWorld_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : world_control(_init)
  {
    (void)_init;
  }

  explicit ControlWorld_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : world_control(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _world_control_type =
    ros_gz_interfaces::msg::WorldControl_<ContainerAllocator>;
  _world_control_type world_control;

  // setters for named parameter idiom
  Type & set__world_control(
    const ros_gz_interfaces::msg::WorldControl_<ContainerAllocator> & _arg)
  {
    this->world_control = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Request
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Request
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlWorld_Request_ & other) const
  {
    if (this->world_control != other.world_control) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlWorld_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlWorld_Request_

// alias to use template instance with default allocator
using ControlWorld_Request =
  ros_gz_interfaces::srv::ControlWorld_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_gz_interfaces


#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Response __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct ControlWorld_Response_
{
  using Type = ControlWorld_Response_<ContainerAllocator>;

  explicit ControlWorld_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit ControlWorld_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  // field types and members
  using _success_type =
    bool;
  _success_type success;

  // setters for named parameter idiom
  Type & set__success(
    const bool & _arg)
  {
    this->success = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Response
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__srv__ControlWorld_Response
    std::shared_ptr<ros_gz_interfaces::srv::ControlWorld_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ControlWorld_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const ControlWorld_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ControlWorld_Response_

// alias to use template instance with default allocator
using ControlWorld_Response =
  ros_gz_interfaces::srv::ControlWorld_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_gz_interfaces

namespace ros_gz_interfaces
{

namespace srv
{

struct ControlWorld
{
  using Request = ros_gz_interfaces::srv::ControlWorld_Request;
  using Response = ros_gz_interfaces::srv::ControlWorld_Response;
};

}  // namespace srv

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__SRV__DETAIL__CONTROL_WORLD__STRUCT_HPP_
