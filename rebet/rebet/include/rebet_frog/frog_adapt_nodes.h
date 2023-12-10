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

namespace BT
{

class AdaptPictureRateOnline: public AdaptOnConditionOnStart<int>
{
  public:

    AdaptPictureRateOnline(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<int>(name, config, AdaptationTarget::BlackboardEntry, AdaptationType::Online)
    {
      _condition_default = true;
      registerAdaptations();

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnStart::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }


};

class AdaptChargeConditionOffline: public AdaptOnConditionOnStart<std::string>
{
  public:

    AdaptChargeConditionOffline(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<std::string>(name, config, AdaptationTarget::BlackboardEntry, AdaptationType::Offline)
    {
      _condition_default = true;
      sum_power_metric_observed = 0.0;
      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnStart::providedPorts();

      PortsList child_ports =  {
            InputPort<double>(METRIC_TO_CHECK, "power_qr value")
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual bool evaluate_condition() override
    {
      _offline_adaptations = {};
      //We assume, that since the QR from which this node receives Input should always output to its port prior to this being ticked, that the state is up to date.
      double curr_power_metric;
      auto res = getInput<double>(METRIC_TO_CHECK, curr_power_metric);

      


      if(res)
      {
        sum_power_metric_observed+=curr_power_metric;
        if(sum_power_metric_observed > CHARGE_THRESHOLD) //
        {
          RCLCPP_INFO(node_->get_logger(), "\n\n\nPower Tresh Exceed!\n\n\n");
          sum_power_metric_observed = 0.0;

          std::string param_name;
          getInput(ADAP_SUB, param_name);
          rebet_msgs::msg::Adaptation adap;
          rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue("charge"));
          adap.adaptation_type = static_cast<int8_t>(AdaptationTarget::BlackboardEntry);
          adap.blackboard_adaptation = adap_param.to_parameter_msg();

          _offline_adaptations.push_back(adap);

    
          return true;
        }
      }

      return false;
    }

  private:
    static constexpr const char* METRIC_TO_CHECK = "power_consumed";
    double sum_power_metric_observed;
    const double CHARGE_THRESHOLD = 100.0;
};


class AdaptMaxSpeedOnline : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptMaxSpeedOnline(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::Online)
    {
      //Since we are only interested in modifying the x-axis (backwards/forwards) speed, we wrap the adaptation_options given into the required triple of x y theta speeds.
        std::vector<double> param_values;
        std::string param_name;
        std::string node_name;
        getInput(ADAP_OPT, param_values);
        getInput(ADAP_SUB, param_name);
        getInput(ADAP_LOC, node_name);

        

        rebet_msgs::msg::AdaptationOptions variable_param = rebet_msgs::msg::AdaptationOptions();


        variable_param.name = param_name;
        variable_param.node_name = node_name;
        variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

        for (double val : param_values) {
            std::vector<double> speed_vector = {val,_default_y_velocity,_default_theta_velocity};
            rclcpp::ParameterValue par_val = rclcpp::ParameterValue(speed_vector); //Here we wrap it with default values
            variable_param.possible_values.push_back(par_val.to_value_msg());
        }
        _var_params.push_back(variable_param); //vector of VariableParameter   

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

  private:
    double _default_y_velocity = 0.0; //The robot in question can not move along its y axis independently.
    double _default_theta_velocity = 2.5; //We are not interested in modifying the rotation speed here, but could be extended to do so.


};



}   // namespace BT
