// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ros_gz_interfaces:msg/SensorNoise.idl
// generated code does not contain a copyright notice

#ifndef ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__STRUCT_HPP_
#define ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ros_gz_interfaces__msg__SensorNoise __attribute__((deprecated))
#else
# define DEPRECATED__ros_gz_interfaces__msg__SensorNoise __declspec(deprecated)
#endif

namespace ros_gz_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct SensorNoise_
{
  using Type = SensorNoise_<ContainerAllocator>;

  explicit SensorNoise_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->mean = 0.0;
      this->stddev = 0.0;
      this->bias_mean = 0.0;
      this->bias_stddev = 0.0;
      this->precision = 0.0;
      this->dynamic_bias_stddev = 0.0;
      this->dynamic_bias_correlation_time = 0.0;
    }
  }

  explicit SensorNoise_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->type = 0;
      this->mean = 0.0;
      this->stddev = 0.0;
      this->bias_mean = 0.0;
      this->bias_stddev = 0.0;
      this->precision = 0.0;
      this->dynamic_bias_stddev = 0.0;
      this->dynamic_bias_correlation_time = 0.0;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _type_type =
    uint8_t;
  _type_type type;
  using _mean_type =
    double;
  _mean_type mean;
  using _stddev_type =
    double;
  _stddev_type stddev;
  using _bias_mean_type =
    double;
  _bias_mean_type bias_mean;
  using _bias_stddev_type =
    double;
  _bias_stddev_type bias_stddev;
  using _precision_type =
    double;
  _precision_type precision;
  using _dynamic_bias_stddev_type =
    double;
  _dynamic_bias_stddev_type dynamic_bias_stddev;
  using _dynamic_bias_correlation_time_type =
    double;
  _dynamic_bias_correlation_time_type dynamic_bias_correlation_time;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__type(
    const uint8_t & _arg)
  {
    this->type = _arg;
    return *this;
  }
  Type & set__mean(
    const double & _arg)
  {
    this->mean = _arg;
    return *this;
  }
  Type & set__stddev(
    const double & _arg)
  {
    this->stddev = _arg;
    return *this;
  }
  Type & set__bias_mean(
    const double & _arg)
  {
    this->bias_mean = _arg;
    return *this;
  }
  Type & set__bias_stddev(
    const double & _arg)
  {
    this->bias_stddev = _arg;
    return *this;
  }
  Type & set__precision(
    const double & _arg)
  {
    this->precision = _arg;
    return *this;
  }
  Type & set__dynamic_bias_stddev(
    const double & _arg)
  {
    this->dynamic_bias_stddev = _arg;
    return *this;
  }
  Type & set__dynamic_bias_correlation_time(
    const double & _arg)
  {
    this->dynamic_bias_correlation_time = _arg;
    return *this;
  }

  // constant declarations
  static constexpr uint8_t NONE =
    0u;
  static constexpr uint8_t GAUSSIAN =
    2u;
  static constexpr uint8_t GAUSSIAN_QUANTIZED =
    3u;

  // pointer types
  using RawPtr =
    ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator> *;
  using ConstRawPtr =
    const ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ros_gz_interfaces__msg__SensorNoise
    std::shared_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ros_gz_interfaces__msg__SensorNoise
    std::shared_ptr<ros_gz_interfaces::msg::SensorNoise_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const SensorNoise_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->type != other.type) {
      return false;
    }
    if (this->mean != other.mean) {
      return false;
    }
    if (this->stddev != other.stddev) {
      return false;
    }
    if (this->bias_mean != other.bias_mean) {
      return false;
    }
    if (this->bias_stddev != other.bias_stddev) {
      return false;
    }
    if (this->precision != other.precision) {
      return false;
    }
    if (this->dynamic_bias_stddev != other.dynamic_bias_stddev) {
      return false;
    }
    if (this->dynamic_bias_correlation_time != other.dynamic_bias_correlation_time) {
      return false;
    }
    return true;
  }
  bool operator!=(const SensorNoise_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct SensorNoise_

// alias to use template instance with default allocator
using SensorNoise =
  ros_gz_interfaces::msg::SensorNoise_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorNoise_<ContainerAllocator>::NONE;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorNoise_<ContainerAllocator>::GAUSSIAN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr uint8_t SensorNoise_<ContainerAllocator>::GAUSSIAN_QUANTIZED;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace ros_gz_interfaces

#endif  // ROS_GZ_INTERFACES__MSG__DETAIL__SENSOR_NOISE__STRUCT_HPP_
