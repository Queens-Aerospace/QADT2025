// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:srv/SpawnEntity.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__SRV__DETAIL__SPAWN_ENTITY__STRUCT_HPP_
#define ROS_GZ_INTERFACES__SRV__DETAIL__SPAWN_ENTITY__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'entity_factory'
#include "ros_gz_interfaces/msg/detail/entity_factory__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Request __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Request __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SpawnEntity_Request_
{
  using Type = SpawnEntity_Request_<ContainerAllocator>;

  explicit SpawnEntity_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : entity_factory(_init)
  {
    (void)_init;
  }

  explicit SpawnEntity_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : entity_factory(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _entity_factory_type =
    ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator>;
  _entity_factory_type entity_factory;

  // setters for named parameter idiom
  Type & set__entity_factory(
    const ros_gz_interfaces::msg::EntityFactory_<ContainerAllocator> & _arg)
  {
    this->entity_factory = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Request
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Request
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpawnEntity_Request_ & other) const
  {
    if (this->entity_factory != other.entity_factory) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpawnEntity_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpawnEntity_Request_

// alias to use template instance with default allocator
using SpawnEntity_Request =
  ros_gz_interfaces::srv::SpawnEntity_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_gz_interfaces


#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Response __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Response __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct SpawnEntity_Response_
{
  using Type = SpawnEntity_Response_<ContainerAllocator>;

  explicit SpawnEntity_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->success = false;
    }
  }

  explicit SpawnEntity_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
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
    ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Response
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__srv__SpawnEntity_Response
    std::shared_ptr<ros_gz_interfaces::srv::SpawnEntity_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SpawnEntity_Response_ & other) const
  {
    if (this->success != other.success) {
      return false;
    }
    return true;
  }
  bool operator!=(const SpawnEntity_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SpawnEntity_Response_

// alias to use template instance with default allocator
using SpawnEntity_Response =
  ros_gz_interfaces::srv::SpawnEntity_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace ros_gz_interfaces

namespace ros_gz_interfaces
{

namespace srv
{

struct SpawnEntity
{
  using Request = ros_gz_interfaces::srv::SpawnEntity_Request;
  using Response = ros_gz_interfaces::srv::SpawnEntity_Response;
};

}  // namespace srv

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__SRV__DETAIL__SPAWN_ENTITY__STRUCT_HPP_
