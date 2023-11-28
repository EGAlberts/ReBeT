#ifndef rebet__PARAMETER_VALUE_HPP_
#define rebet__PARAMETER_VALUE_HPP_

#include <exception>
#include <iostream>
#include <ostream>
#include <sstream>
#include <string>
#include <vector>

#include "rebet_msgs/msg/system_attribute_value.hpp"
#include "rebet_msgs/msg/system_attribute_type.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "sensor_msgs/msg/laser_scan  .hpp"


//Inspired by rclcpp/parameter_value.hpp

namespace rebet
{

enum SystemAttributeType : uint8_t
{
  ATTRIBUTE_NOT_SET = rebet_msgs::msg::SystemAttributeType::ATTRIBUTE_NOT_SET,
  ATTRIBUTE_ODOM    = rebet_msgs::msg::SystemAttributeType::ATTRIBUTE_ODOM,
  ATTRIBUTE_DIAG    = rebet_msgs::msg::SystemAttributeType::ATTRIBUTE_DIAG,
  ATTRIBUTE_LASER    = rebet_msgs::msg::SystemAttributeType::ATTRIBUTE_LASER,


};

/// Return the name of a parameter type
std::string
to_string(SystemAttributeType type);

std::ostream &
operator<<(std::ostream & os, SystemAttributeType type);

class SystemAttributeTypeException : public std::runtime_error
{
public:
  /// Construct an instance.
  /**
   * \param[in] expected the expected parameter type.
   * \param[in] actual the actual parameter type.
   */
  SystemAttributeTypeException(SystemAttributeType expected, SystemAttributeType actual)
  : std::runtime_error("expected [" + to_string(expected) + "] got [" + to_string(actual) + "]")
  {}
};

/// A class wrapper around the SystemAttributeValue MSG we've defined
class SystemAttributeValue
{
public:
  /// Construct a parameter value with type PARAMETER_NOT_SET.
  
  SystemAttributeValue();
  
  /// Construct a parameter value from a message.
  explicit SystemAttributeValue(const rebet_msgs::msg::SystemAttributeValue & value);
  /// Construct a parameter value with type PARAMETER_BOOL.
  explicit SystemAttributeValue(const nav_msgs::msg::Odometry odom_value);

  explicit SystemAttributeValue(const diagnostic_msgs::msg::KeyValue diag_value);

  explicit SystemAttributeValue(const sensor_msgs::msg::LaserScan laser_value);



  /// Return an enum indicating the type of the set value.
  
  SystemAttributeType
  get_type() const;

  /// Return a message populated with the parameter value
  
  rebet_msgs::msg::SystemAttributeValue
  to_value_msg() const;

  /// Equal operator.
  
  bool
  operator==(const SystemAttributeValue & rhs) const;

  /// Not equal operator.
  
  bool
  operator!=(const SystemAttributeValue & rhs) const;

  // The following get() variants require the use of SystemAttributeType

  template<SystemAttributeType type>
  constexpr
  typename std::enable_if<type == SystemAttributeType::ATTRIBUTE_ODOM, const nav_msgs::msg::Odometry &>::type
  get() const
  {
    if (value_.type != rebet_msgs::msg::SystemAttributeType::ATTRIBUTE_ODOM) {
      throw SystemAttributeTypeException(SystemAttributeType::ATTRIBUTE_ODOM, get_type());
    }
    return value_.odom_value;
  }

  template<SystemAttributeType type>
  constexpr
  typename std::enable_if<type == SystemAttributeType::ATTRIBUTE_DIAG, const diagnostic_msgs::msg::KeyValue &>::type
  get() const
  {
    if (value_.type != rebet_msgs::msg::SystemAttributeType::ATTRIBUTE_DIAG) {
      throw SystemAttributeTypeException(SystemAttributeType::ATTRIBUTE_DIAG, get_type());
    }
    return value_.diag_value;
  }

  template<SystemAttributeType type>
  constexpr
  typename std::enable_if<type == SystemAttributeType::ATTRIBUTE_LASER, const sensor_msgs::msg::LaserScan &>::type
  get() const
  {
    if (value_.type != rebet_msgs::msg::SystemAttributeType::ATTRIBUTE_LASER) {
      throw SystemAttributeTypeException(SystemAttributeType::ATTRIBUTE_LASER, get_type());
    }
    return value_.laser_value;
  }

  
private:
  rebet_msgs::msg::SystemAttributeValue value_;
};

/// Return the value of a parameter as a string

std::string
to_string(const SystemAttributeValue & type);


} //namespace rebet

#endif  // rebet__PARAMETER_VALUE_HPP_