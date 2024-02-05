#pragma once

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rebet_msgs/msg/variable_parameters.hpp"
#include "rebet_msgs/msg/variable_parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rebet/adapt_node.h"
#include "rebet/rebet_utilities.hpp"



namespace BT
{

class AdaptSpiralAltitudeOnline : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptSpiralAltitudeOnline(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::Online)
    {
      registerAdaptations();
      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.
  
    }

    virtual double utility_of_adaptation(rcl_interfaces::msg::Parameter ros_parameter) override
    {
      return 1.0;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

};

class AdaptSpiralAltitudeOffline : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptSpiralAltitudeOffline(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::Offline)
    {

    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
        InputPort<double>(SEARCH_EFF, "how efficient search would be right now."),

              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    bool altitude_adaptation_msg(double altitude_value)
    {

      _offline_adaptations = {};
      if(altitude_value == _current_altitude)
      {
        //Already at said altitude
        return false;
      }


      _current_altitude = altitude_value;

      std::string param_name;
      std::string node_name;

      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);

      rebet_msgs::msg::Adaptation adap;
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(_current_altitude));
      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _offline_adaptations.push_back(adap);
        std::cout << "altitude adjustment " << altitude_value << "\n\n" << std::endl;
        std::cout << "altitude adjustment " << adap_param.to_parameter_msg().value.double_value << "\n\n" << std::endl;



      return true;
    }

    virtual bool evaluate_condition() override
    {
      // std::cout << "I'm here in offline altitude adjustment! 1\n\n" << std::endl;

      if(AdaptPeriodicallyOnRunning::evaluate_condition())
      {
        std::cout << "I'm here in offline altitude adjustment! 2\n\n" << std::endl;

        double current_search;

        auto search_res = getInput(SEARCH_EFF, current_search);






        std::cout << "I'm here in offline altitude adjustment!  3 \n\n" << std::endl;
        if(search_res)
        {
          std::cout << "now here curr_search " << current_search << std::endl;

          if(current_search >= 3.25) // 3.25 (from SUAVE) normalized
          {
            return altitude_adaptation_msg(ALT_HIGH);
          }
          else if(current_search >= 2.25)  // 2.25 (from SUAVE) normalized
          {
            return altitude_adaptation_msg(ALT_MED);
          }
          else
          {
            return altitude_adaptation_msg(ALT_LOW);
          }
          
        }
      }
      return false;    
    }

    private:
      static constexpr const char* SEARCH_EFF = "search_efficiency";
      const double ALT_HIGH = 3.0;
      const double ALT_MED = 2.0;
      const double ALT_LOW = 1.0;
      double _current_altitude = ALT_LOW;


};

class AdaptThrusterOffline : public AdaptOnConditionOnRunning<double>
{

  public:
    AdaptThrusterOffline(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnRunning<double>(name, config, AdaptationTarget::LifecycleTransition, AdaptationType::Offline)
    {
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnRunning::providedPorts();

      PortsList child_ports =  {
        InputPort<std::string>(METRIC_TO_CHECK, "status of thruster"),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual bool evaluate_condition() override
    {
      _offline_adaptations = {};
      //We assume, that since the QR from which this node receives Input should always output to its port prior to this being ticked, that the state is up to date.
      std::string thruster_eval;
      auto res = getInput<std::string>(METRIC_TO_CHECK, thruster_eval);

      std::string node_name;
      getInput(ADAP_LOC, node_name);

      if(res)
      {
        if(thruster_eval == "Thruster Failure") //It may mean that we should try to recover the thrusters
        {
          lifecycle_msgs::msg::Transition transition;
          transition.id = 3; //Activate transition

          rebet_msgs::msg::Adaptation adap;

          adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::LifecycleTransition);
          adap.lifecycle_adaptation = transition;
          adap.node_name = node_name;

          _offline_adaptations.push_back(adap);

    
          return true;
        }
        if(thruster_eval == "Thrusters Recovered")
        {
          lifecycle_msgs::msg::Transition transition;
          transition.id = 4; //Deactivate transition

          rebet_msgs::msg::Adaptation adap;

          adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::LifecycleTransition);
          adap.lifecycle_adaptation = transition;
          adap.node_name = node_name;

          _offline_adaptations.push_back(adap);

          return true;
        }
      }

      return false;
    }
  private:
      static constexpr const char* METRIC_TO_CHECK = "thruster_condition";
};


}   // namespace BT
