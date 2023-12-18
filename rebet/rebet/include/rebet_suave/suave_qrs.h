#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rebet_msgs/msg/objects_identified.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rebet/qr_node.h"

#include "rebet_frog/frog_constants.hpp"

namespace BT
{
/**
 * @brief The QRNode is used to constrain variable action nodes it decorates.
 *
 *
 * 
 *
 * 
 * 
 * 
 * 
 * 
 * Note: If in the future a BT should be designed with multiple instances of the same QR, it would become impractical. Right now the only feasible case it having two QRs which are never active simultaneously. 
 * In all other cases it would require creating a new (very similar) subclass as the linked blackboard entries would otherwise cause issue.
 */


class MoveRobustly : public TaskLevelQR
{
  public:
    MoveRobustly(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::TaskEfficiency)
    {


    }

    static PortsList providedPorts()
    {
      PortsList base_ports = QRNode::providedPorts();

      PortsList child_ports =  {
              InputPort<rebet::SystemAttributeValue>(STATUS_ONE, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_TWO, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_THREE, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_FOUR, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_FIVE, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_SIX, "status of thruster"),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    void check_thrusters()
    {
      int recovery_count = 0; 
      for(unsigned int i = 0; i < thruster_stati.size(); i++)
      {
        auto status_portname = thruster_stati[i];

      
        rebet::SystemAttributeValue thruster_status_att; 
        auto res = getInput(status_portname, thruster_status_att);

        if(res)
        {
          auto as_sysatt_msg = thruster_status_att.to_value_msg();
          if(as_sysatt_msg.header.stamp != thruster_stati_timestamps[i])
          {
            thruster_stati_timestamps[i] = as_sysatt_msg.header.stamp;
            diagnostic_msgs::msg::KeyValue keyvalue_msg = thruster_status_att.get<rebet::SystemAttributeType::ATTRIBUTE_DIAG>();
            
            if(keyvalue_msg.value == "FALSE")
            {
              std::cout << "\n\n\n\n\n\nCondition MET Because of failure!!\n\n\n\n\n" << std::endl;
              setOutput(QR_STATE, "Thruster Failure");
            }
            else if(keyvalue_msg.value == "RECOVERED")
            {
              //The way SUAVE works is that it recovers all thrusters no matter how many broke, so this condition is only met when for each thruster I hear recovered once.
              recovery_count++;
              std::cout << "recovery count " << recovery_count;
              if(recovery_count == 5)
              {
                recovery_count = 0;
                std::cout << "\n\n\n\n\n\nCondition MET Because of recovered!!\n\n\n\n\n" << std::endl;
                setOutput(QR_STATE, "Thrusters Recovered");              
              }
            }
          }
        }

      }
    }

    virtual void calculate_measure() override
    {
      setOutput(QR_STATE, "OK");
      check_thrusters();
    }

    private:
    
      static constexpr const char* STATUS_ONE = "thruster_status_one";
      static constexpr const char* STATUS_TWO = "thruster_status_two";
      static constexpr const char* STATUS_THREE = "thruster_status_three";
      static constexpr const char* STATUS_FOUR = "thruster_status_four";
      static constexpr const char* STATUS_FIVE = "thruster_status_five";
      static constexpr const char* STATUS_SIX = "thruster_status_six";

      std::vector<std::string> thruster_stati = {STATUS_ONE, STATUS_TWO, STATUS_THREE, STATUS_FOUR, STATUS_FIVE, STATUS_SIX};
      std::vector<builtin_interfaces::msg::Time> thruster_stati_timestamps = {builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time()};
};




class SearchEfficiently : public TaskLevelQR
{
  public:
    SearchEfficiently(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::TaskEfficiency)
    {
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      _last_timestamp = builtin_interfaces::msg::Time();

      _water_visibilities = {};

      
  
    }

    void initialize(int window_length)
    {
        _window_length = window_length;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = QRNode::providedPorts();

      PortsList child_ports =  {
              InputPort<rebet::SystemAttributeValue>(IN_WATER,"water_visibility message wrapped in keyvalue message wrapped in a systemattributevalue instance"),

              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {

      auto res = getInput(IN_WATER,_watervis_attribute); 

      if(res)
      {
        auto as_sysatt_msg = _watervis_attribute.to_value_msg();
        if(as_sysatt_msg.header.stamp != _last_timestamp)
        {
          _keyvalue_msg = _watervis_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_DIAG>();
          float water_visibility = std::stof(_keyvalue_msg.value);

          // _water_visibilities.push_back(water_visibility);

          _last_timestamp = as_sysatt_msg.header.stamp;


          _metric =  water_visibility;


          output_metric();
          metric_mean();

          setOutput(MEAN_METRIC,_average_metric);

          std::cout << "new water vis received " << water_visibility; 
        }
      }

      

      // auto curr_time_pointer = std::chrono::system_clock::now();
      
      // int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      // int elapsed_seconds = current_time-_window_start;
      // if(elapsed_seconds >= _window_length)
      // {
      //   float wvis_sum = std::accumulate(_water_visibilities.begin(), _water_visibilities.end(), 0);
      //   float average_water_vis = wvis_sum / (float)_water_visibilities.size();
      //   _metric = (average_water_vis - _min_wvis) / (_max_wvis - _min_wvis);

      //   output_metric();
      //   metric_mean();
      //   setOutput(MEAN_METRIC,_average_metric);
      //   _window_start = current_time;
      //   _water_visibilities = {};
      // }

    }

    private:
      rebet::SystemAttributeValue _watervis_attribute;
      diagnostic_msgs::msg::KeyValue _keyvalue_msg;


      std::vector<float> _water_visibilities;

      builtin_interfaces::msg::Time _last_timestamp;

      int _window_length;
      int _window_start;
      int _coun_max_wvister;
      float _max_wvis = 3.75; //This is a parameter for suave.. I'd say it should be linked, but ideally I'd just dynamically bound it instead.
      float _min_wvis = 1.25; //idem ditto


      static constexpr const char* IN_WATER = "in_water_visibility";
      static constexpr const char* STATUS_ONE = "thruster_status_one";
      static constexpr const char* STATUS_TWO = "thruster_status_two";
      static constexpr const char* STATUS_THREE = "thruster_status_three";
      static constexpr const char* STATUS_FOUR = "thruster_status_four";
      static constexpr const char* STATUS_FIVE = "thruster_status_five";
      static constexpr const char* STATUS_SIX = "thruster_status_six";

      std::vector<std::string> thruster_stati = {STATUS_ONE, STATUS_TWO, STATUS_THREE, STATUS_FOUR, STATUS_FIVE, STATUS_SIX};
      std::vector<builtin_interfaces::msg::Time> thruster_stati_timestamps = {builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time()};
};


}   // namespace BT
