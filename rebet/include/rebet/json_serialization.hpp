#ifndef rebet__REBET_JSON_SERIALIZATION_HPP_
#define rebet__REBET_JSON_SERIALIZATION_HPP_

#include <nlohmann/json.hpp>
#include "behaviortree_cpp/json_export.h"
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "rebet_msgs/msg/qr.hpp"

namespace nlohmann
{

template<>
struct adl_serializer<geometry_msgs::msg::Point>
{
  static void to_json(nlohmann::json & j, const geometry_msgs::msg::Point & point)
  {
    j = {{"x", point.x}, {"y", point.y}, {"z", point.z}};
  }

  static void from_json(const nlohmann::json & j, geometry_msgs::msg::Point & point)
  {
    j.at("x").get_to(point.x);
    j.at("y").get_to(point.y);
    j.at("z").get_to(point.z);
  }
};

template<>
struct adl_serializer<geometry_msgs::msg::Quaternion>
{
  static void to_json(nlohmann::json & j, const geometry_msgs::msg::Quaternion & quat)
  {
    j = {{"x", quat.x}, {"y", quat.y}, {"z", quat.z}, {"w", quat.w}};
  }

  static void from_json(const nlohmann::json & j, geometry_msgs::msg::Quaternion & quat)
  {
    j.at("x").get_to(quat.x);
    j.at("y").get_to(quat.y);
    j.at("z").get_to(quat.z);
    j.at("w").get_to(quat.w);
  }
};

template<>
struct adl_serializer<geometry_msgs::msg::Pose>
{
  static void to_json(nlohmann::json & j, const geometry_msgs::msg::Pose & pose)
  {
    j = {{"position", pose.position}, {"orientation", pose.orientation}};
  }

  static void from_json(const nlohmann::json & j, geometry_msgs::msg::Pose & pose)
  {
    j.at("position").get_to(pose.position);
    j.at("orientation").get_to(pose.orientation);
  }
};

template<>
struct adl_serializer<geometry_msgs::msg::Twist>
{
  static void to_json(nlohmann::json & j, const geometry_msgs::msg::Twist & twist)
  {
    j = {
      {"linear", {{"x", twist.linear.x}, {"y", twist.linear.y}, {"z", twist.linear.z}}},
      {"angular", {{"x", twist.angular.x}, {"y", twist.angular.y}, {"z", twist.angular.z}}}
    };
  }

  static void from_json(const nlohmann::json & j, geometry_msgs::msg::Twist & twist)
  {
    j.at("linear").at("x").get_to(twist.linear.x);
    j.at("linear").at("y").get_to(twist.linear.y);
    j.at("linear").at("z").get_to(twist.linear.z);
    j.at("angular").at("x").get_to(twist.angular.x);
    j.at("angular").at("y").get_to(twist.angular.y);
    j.at("angular").at("z").get_to(twist.angular.z);
  }
};

template<>
struct adl_serializer<nav_msgs::msg::Odometry>
{
  static void to_json(nlohmann::json & j, const nav_msgs::msg::Odometry & odom)
  {
    j = {
      {"header", {
          {"stamp", {
              {"sec", odom.header.stamp.sec},
              {"nanosec", odom.header.stamp.nanosec}
            }},
          {"frame_id", odom.header.frame_id}
        }},
      {"child_frame_id", odom.child_frame_id},
      {"pose", {
          {"pose", odom.pose.pose},
          {"covariance", odom.pose.covariance}
        }},
      {"twist", {
          {"twist", odom.twist.twist},
          {"covariance", odom.twist.covariance}
        }}
    };
  }

  static void from_json(const nlohmann::json & j, nav_msgs::msg::Odometry & odom)
  {
    j.at("header").at("stamp").at("sec").get_to(odom.header.stamp.sec);
    j.at("header").at("stamp").at("nanosec").get_to(odom.header.stamp.nanosec);
    j.at("header").at("frame_id").get_to(odom.header.frame_id);
    j.at("child_frame_id").get_to(odom.child_frame_id);

    j.at("pose").at("pose").get_to(odom.pose.pose);
    j.at("pose").at("covariance").get_to(odom.pose.covariance);

    j.at("twist").at("twist").get_to(odom.twist.twist);
    j.at("twist").at("covariance").get_to(odom.twist.covariance);
  }
};

template<>
struct adl_serializer<std_msgs::msg::Float32>
{
  static void to_json(nlohmann::json & j, const std_msgs::msg::Float32 & msg)
  {
    j = {{"data", msg.data}};
  }

  static void from_json(const nlohmann::json & j, std_msgs::msg::Float32 & msg)
  {
    j.at("data").get_to(msg.data);
  }
};

template<>
struct adl_serializer<std_msgs::msg::String>
{
  static void to_json(nlohmann::json & j, const std_msgs::msg::String & msg)
  {
    j = {{"data", msg.data}};
  }

  static void from_json(const nlohmann::json & j, std_msgs::msg::String & msg)
  {
    j.at("data").get_to(msg.data);
  }
};

template<>
struct adl_serializer<std_msgs::msg::Bool>
{
  static void to_json(nlohmann::json & j, const std_msgs::msg::Bool & msg)
  {
    j = {{"data", msg.data}};
  }

  static void from_json(const nlohmann::json & j, std_msgs::msg::Bool & msg)
  {
    j.at("data").get_to(msg.data);
  }
};

template<>
struct adl_serializer<sensor_msgs::msg::LaserScan>
{
  static void to_json(nlohmann::json & j, const sensor_msgs::msg::LaserScan & scan)
  {
    j = {
      {"header", {
          {"stamp", {
              {"sec", scan.header.stamp.sec},
              {"nanosec", scan.header.stamp.nanosec}
            }},
          {"frame_id", scan.header.frame_id}
        }},
      {"angle_min", scan.angle_min},
      {"angle_max", scan.angle_max},
      {"angle_increment", scan.angle_increment},
      {"time_increment", scan.time_increment},
      {"scan_time", scan.scan_time},
      {"range_min", scan.range_min},
      {"range_max", scan.range_max},
      {"ranges", scan.ranges},
      {"intensities", scan.intensities}
    };
  }

  static void from_json(const nlohmann::json & j, sensor_msgs::msg::LaserScan & scan)
  {
    j.at("header").at("stamp").at("sec").get_to(scan.header.stamp.sec);
    j.at("header").at("stamp").at("nanosec").get_to(scan.header.stamp.nanosec);
    j.at("header").at("frame_id").get_to(scan.header.frame_id);

    j.at("angle_min").get_to(scan.angle_min);
    j.at("angle_max").get_to(scan.angle_max);
    j.at("angle_increment").get_to(scan.angle_increment);
    j.at("time_increment").get_to(scan.time_increment);
    j.at("scan_time").get_to(scan.scan_time);
    j.at("range_min").get_to(scan.range_min);
    j.at("range_max").get_to(scan.range_max);
    j.at("ranges").get_to(scan.ranges);
    j.at("intensities").get_to(scan.intensities);
  }
};


template<>
struct adl_serializer<sensor_msgs::msg::Range>
{
  static void to_json(nlohmann::json & j, const sensor_msgs::msg::Range & range)
  {
    j = {
      {"header", {
          {"stamp", {
              {"sec", range.header.stamp.sec},
              {"nanosec", range.header.stamp.nanosec}
            }},
          {"frame_id", range.header.frame_id}
        }},
      {"radiation_type", range.radiation_type},
      {"field_of_view", range.field_of_view},
      {"min_range", range.min_range},
      {"max_range", range.max_range},
      {"range", range.range}
    };
  }

  static void from_json(const nlohmann::json & j, sensor_msgs::msg::Range & range)
  {
    j.at("header").at("stamp").at("sec").get_to(range.header.stamp.sec);
    j.at("header").at("stamp").at("nanosec").get_to(range.header.stamp.nanosec);
    j.at("header").at("frame_id").get_to(range.header.frame_id);

    j.at("radiation_type").get_to(range.radiation_type);
    j.at("field_of_view").get_to(range.field_of_view);
    j.at("min_range").get_to(range.min_range);
    j.at("max_range").get_to(range.max_range);
    j.at("range").get_to(range.range); 
  }
};

template<>
struct adl_serializer<sensor_msgs::msg::BatteryState>
{
  static void to_json(nlohmann::json & j, const sensor_msgs::msg::BatteryState & battery_state)
  {
    j = {
      {"header", {
          {"stamp", {
              {"sec", battery_state.header.stamp.sec},
              {"nanosec", battery_state.header.stamp.nanosec}
            }},
          {"frame_id", battery_state.header.frame_id}
        }},
      {"voltage", battery_state.voltage},
      {"current", battery_state.current},
      {"charge", battery_state.charge},
      {"capacity", battery_state.capacity},
      {"design_capacity", battery_state.design_capacity},
      {"percentage", battery_state.percentage},
      {"power_supply_status", battery_state.power_supply_status},
      {"power_supply_health", battery_state.power_supply_health},
      {"power_supply_technology", battery_state.power_supply_technology},
      {"present", battery_state.present},
      {"cell_voltage", battery_state.cell_voltage},
      {"cell_temperature", battery_state.cell_temperature},
      {"location", battery_state.location},
      {"serial_number", battery_state.serial_number}
    };
  }

  static void from_json(const nlohmann::json & j, sensor_msgs::msg::BatteryState & battery_state)
  {
    j.at("header").at("stamp").at("sec").get_to(battery_state.header.stamp.sec);
    j.at("header").at("stamp").at("nanosec").get_to(battery_state.header.stamp.nanosec);
    j.at("header").at("frame_id").get_to(battery_state.header.frame_id);

    j.at("voltage").get_to(battery_state.voltage);
    j.at("current").get_to(battery_state.current);
    j.at("charge").get_to(battery_state.charge);
    j.at("capacity").get_to(battery_state.capacity);
    j.at("design_capacity").get_to(battery_state.design_capacity);
    j.at("percentage").get_to(battery_state.percentage);
    j.at("power_supply_status").get_to(battery_state.power_supply_status);
    j.at("power_supply_health").get_to(battery_state.power_supply_health);
    j.at("power_supply_technology").get_to(battery_state.power_supply_technology);
    j.at("present").get_to(battery_state.present);
    j.at("cell_voltage").get_to(battery_state.cell_voltage);  
    j.at("cell_temperature").get_to(battery_state.cell_temperature);
    j.at("location").get_to(battery_state.location);
    j.at("serial_number").get_to(battery_state.serial_number);
  }
};

template<>
struct adl_serializer<diagnostic_msgs::msg::KeyValue>
{
  static void to_json(nlohmann::json & j, const diagnostic_msgs::msg::KeyValue & msg)
  {
    j = {
      {"key", msg.key},
      {"value", msg.value}
    };
  }

  static void from_json(const nlohmann::json & j, diagnostic_msgs::msg::KeyValue & msg)
  {
    j.at("key").get_to(msg.key);
    j.at("value").get_to(msg.value);
  }
};

template<>
struct adl_serializer<rebet::SystemAttributeValue>
{
  static void to_json(nlohmann::json & j, const rebet::SystemAttributeValue & sys_attr_val)
  {
    auto message_type = static_cast<int>(sys_attr_val.get_type());

    switch (sys_attr_val.get_type()) {
      case rebet::SystemAttributeType::ATTRIBUTE_ODOM:
        j = {
          {"system_attribute_value", {
              {"odometry_message", sys_attr_val.to_value_msg().odom_value},        // Ensure JSON conversion
              {"message_type", message_type}
            }}
        };
        break;

      case rebet::SystemAttributeType::ATTRIBUTE_DIAG:
        j = {
          {"system_attribute_value", {
              {"diagnostics_message", sys_attr_val.to_value_msg().diag_value},
              {"message_type", message_type}
            }}
        };
        break;

      case rebet::SystemAttributeType::ATTRIBUTE_LASER:
        j = {
          {"system_attribute_value", {
              {"laserscan_message", sys_attr_val.to_value_msg().laser_value},
              {"message_type", message_type}
            }}
        };
        break;

      case rebet::SystemAttributeType::ATTRIBUTE_FLOAT:
        j = {
          {"system_attribute_value", {
              {"float_message", sys_attr_val.to_value_msg().float_value},        // Assuming it's directly serializable
              {"message_type", message_type}
            }}
        };
        break;

      case rebet::SystemAttributeType::ATTRIBUTE_NOT_SET:
        j = {
          {"system_attribute_value", {
              {"no_message", "no attribute set within the system attribute message"},
              {"message_type", message_type}
            }}
        };
        break;
      
        case rebet::SystemAttributeType::ATTRIBUTE_RANGE:
        j = {
          {"system_attribute_value", {
              {"range_message", sys_attr_val.to_value_msg().range_value},
              {"message_type", message_type}
            }}
        };
        break;
      case rebet::SystemAttributeType::ATTRIBUTE_BATTERY:
        j = {
          {"system_attribute_value", {
              {"battery_message", sys_attr_val.to_value_msg().battery_value},
              {"message_type", message_type}
            }}
        };
        break;

      default:
        j = {
          {"system_attribute_value", {
              {"unknown_type", "type set to unknown value within the system attribute message"},
              {"message_type", message_type}
            }}
        };
    }
  }

  static void from_json(const nlohmann::json & j, rebet::SystemAttributeValue & sys_attr_val)
  {
    switch (j.at("system_attribute_value").at("message_type").get<int>()) {
      case rebet::SystemAttributeType::ATTRIBUTE_ODOM:
        sys_attr_val = rebet::SystemAttributeValue(
          j.at("system_attribute_value").at(
            "odometry_message").get<nav_msgs::msg::Odometry>());
        break;

      case rebet::SystemAttributeType::ATTRIBUTE_DIAG:
        sys_attr_val = rebet::SystemAttributeValue(
          j.at("system_attribute_value").at(
            "diagnostics_message").get<diagnostic_msgs::msg::KeyValue>());
        break;

      case rebet::SystemAttributeType::ATTRIBUTE_LASER:
        sys_attr_val = rebet::SystemAttributeValue(
          j.at("system_attribute_value").at(
            "laserscan_message").get<sensor_msgs::msg::LaserScan>());
        break;

      case rebet::SystemAttributeType::ATTRIBUTE_FLOAT:
        sys_attr_val = rebet::SystemAttributeValue(
          j.at("system_attribute_value").at(
            "float_message").get<std_msgs::msg::Float32>());
        break;
      case rebet::SystemAttributeType::ATTRIBUTE_NOT_SET:
        sys_attr_val = rebet::SystemAttributeValue();
        break;
      case rebet::SystemAttributeType::ATTRIBUTE_RANGE:
        sys_attr_val = rebet::SystemAttributeValue(
          j.at("system_attribute_value").at(
            "range_message").get<sensor_msgs::msg::Range>());
        break;
      case rebet::SystemAttributeType::ATTRIBUTE_BATTERY:
        sys_attr_val = rebet::SystemAttributeValue(
          j.at("system_attribute_value").at(
            "battery_message").get<sensor_msgs::msg::BatteryState>());
        break;
      default:
        throw std::runtime_error(
          "Unknown type encountered when trying to deserialize SystemAttributeValue");  
    }
  }
};

template<>
struct adl_serializer<rebet_msgs::msg::QR>
{
  static void to_json(nlohmann::json & j, const rebet_msgs::msg::QR & qr_msg)
  {
    j = {
      {"qr_name", qr_msg.qr_name},
      {"metric", qr_msg.metric},
      {"weight", qr_msg.weight},
      {"higher_is_better", qr_msg.higher_is_better},
    };
  }

  static void from_json(const nlohmann::json & j, rebet_msgs::msg::QR & qr_msg)
  {
    j.at("qr_name").get_to(qr_msg.qr_name);
    j.at("metric").get_to(qr_msg.metric);
    j.at("weight").get_to(qr_msg.weight);
    j.at("higher_is_better").get_to(qr_msg.higher_is_better);

  }
};

template<>
struct adl_serializer<std::vector<rebet_msgs::msg::QR>>
{
  static void to_json(nlohmann::json & j, const std::vector<rebet_msgs::msg::QR> & qr_list)
  {
    j = nlohmann::json::array();     // Initialize as a JSON array
    for (const auto & qr : qr_list) {
      j.push_back(qr);       // Will use the existing adl_serializer<rebet_msgs::msg::QR>
    }
  }

  static void from_json(const nlohmann::json & j, std::vector<rebet_msgs::msg::QR> & qr_list)
  {
    qr_list.clear();     // Ensure the vector is empty before inserting elements
    for (const auto & item : j) {
      qr_list.push_back(item.get<rebet_msgs::msg::QR>());       // Convert JSON object to QR message
    }
  }
};

}

struct SerializerRegistration
{
  SerializerRegistration()
  {
    // Register individual types
    BT::RegisterJsonDefinition<geometry_msgs::msg::Point>();
    BT::RegisterJsonDefinition<geometry_msgs::msg::Quaternion>();
    BT::RegisterJsonDefinition<geometry_msgs::msg::Pose>();
    BT::RegisterJsonDefinition<geometry_msgs::msg::Twist>();
    BT::RegisterJsonDefinition<nav_msgs::msg::Odometry>();
    BT::RegisterJsonDefinition<std_msgs::msg::Float32>();
    BT::RegisterJsonDefinition<std_msgs::msg::String>();
    BT::RegisterJsonDefinition<std_msgs::msg::Bool>();
    BT::RegisterJsonDefinition<sensor_msgs::msg::LaserScan>();
    BT::RegisterJsonDefinition<sensor_msgs::msg::Range>();
    BT::RegisterJsonDefinition<sensor_msgs::msg::BatteryState>();
    BT::RegisterJsonDefinition<diagnostic_msgs::msg::KeyValue>();
    BT::RegisterJsonDefinition<rebet::SystemAttributeValue>();
    BT::RegisterJsonDefinition<rebet_msgs::msg::QR>();
    BT::RegisterJsonDefinition<std::vector<rebet_msgs::msg::QR>>();
  }
};

// Define a static instance to trigger registration at program startup
static SerializerRegistration JsonReg;

#endif  // rebet__REBET_JSON_SERIALIZATION_HPP_
