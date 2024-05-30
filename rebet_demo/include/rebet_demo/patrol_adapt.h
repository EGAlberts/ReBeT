#pragma once

#include "rebet_demo/adapt_node.h"

namespace BT
{

class AdaptMaxSpeedExternal : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptMaxSpeedExternal(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::External)
    {
      //Since we are only interested in modifying the x-axis (backwards/forwards) speed, we wrap the adaptation_options given into the required triple of x y theta speeds.
      std::vector<double> param_values;
      std::string param_name;
      std::string node_name;
      getInput(ADAP_OPT, param_values);
      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);

      aal_msgs::msg::AdaptationOptions variable_param = aal_msgs::msg::AdaptationOptions();


      variable_param.name = param_name;
      variable_param.node_name = node_name;
      variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);

      for (double val : param_values) {
          std::vector<double> speed_vector = {val,_default_y_velocity,_default_theta_velocity};
          rclcpp::ParameterValue par_val = rclcpp::ParameterValue(speed_vector); //Here we wrap it with default values
          variable_param.possible_values.push_back(par_val.to_value_msg());
      }
      _var_params.push_back(variable_param); //vector of VariableParameter   
    }

    

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
        InputPort<double>(POW_IN,"the power metric"),
        InputPort<std::string>(SAFE_IN,"the safety qr status"),
        InputPort<double>(MOVE_IN,"the movement efficiency"),
        OutputPort<double>(SPD_OUT,"the current chosen max speed"),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }


    virtual double utility_of_adaptation(rcl_interfaces::msg::Parameter ros_parameter) override
    {
      auto parameter_object = rclcpp::ParameterValue(ros_parameter.value);

      std::vector<double> chosen_speeds = parameter_object.get<std::vector<double>>();

      double chosen_max_speed = chosen_speeds[0];

      setOutput(SPD_OUT, chosen_max_speed);
    
      double current_power, current_move;
      std::string safety_status;

      auto safe_res = getInput(SAFE_IN, safety_status);
      auto pow_res = getInput(POW_IN, current_power);
      auto move_res = getInput(MOVE_IN, current_move);

      if(safe_res && pow_res && move_res)
      {
        //MoveSafely is in violation.
        if((safety_status == "UNSAFE") && chosen_max_speed > 0.10)
        {
          return 0.0;
        }
        else
        {
          double inverse_power = 1/current_power;
          double utility = current_move * inverse_power;

          if(utility < 0.0 || current_move < 0.0 || current_power < 0.0) //can happen if there's no values yet.
          {
            return 0.0;
          }
          return utility;
        }
      }
      else
      {
        std::cout << "For some reason a value was not found in the blackboard" << std::endl;  
        if(!safe_res)
        {
          std::cout << "no safe res" << std::endl;
        }

        if(!pow_res)
        {
          std::cout << "no pow_res" << std::endl;
        }

        if(!move_res)
        {
          std::cout << "no move_res" << std::endl;
        }
        return 0.0;
      }

    }

  private:
    double _default_y_velocity = 0.0; //The robot in question can not move along its y axis independently.
    double _default_theta_velocity = 2.5; //We are not interested in modifying the rotation speed here, but could be extended to do so.
    const double MAX_LIN_VEL = 0.5;
    static constexpr const char* POW_IN = "in_power";
    static constexpr const char* SAFE_IN = "in_safety";
    static constexpr const char* MOVE_IN = "in_movement";
    static constexpr const char* SPD_OUT = "report_speed";
    
};

class AdaptCameraFeedInternal: public AdaptOnConditionOnStart<int>
{
  public:

    AdaptCameraFeedInternal(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<int>(name, config, AdaptationTarget::Connection, AdaptationType::Internal)
    {
      _condition_default = true;
      registerAdaptations();
      current_image_feed = OG_CAMERA_TOPIC;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnStart::providedPorts();

      PortsList child_ports =  {
        InputPort<std::vector<double>>(IN_DETEFF,"the det obj task metric"),
        InputPort<rebet::SystemAttributeValue>(IN_LIGHT,"lighting message wrapped in a systemattributevalue instance"),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    bool change_camera_feed(std::string new_topic_name)
    {
      if(current_image_feed == new_topic_name){ return false; }

      std::string param_name;
      std::string node_name;

      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);

      aal_msgs::msg::Adaptation adap;
      
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(new_topic_name));

      std::cout << "type name " << adap_param.get_type_name() << std::endl;

      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _internal_adaptations.push_back(adap);

      current_image_feed = new_topic_name;

      return true;
    }



    virtual bool evaluate_condition() override
    {     
      auto curr_light_res = getInput(IN_LIGHT,_light_attribute);
      auto task_res = getInput(IN_DETEFF, _deteff_metrics);

      if(!task_res)
      {
        std::cout << "no task res" << std::endl;
      }
      if(!curr_light_res)
      {
        std::cout << "no currlight res" << std::endl;
      }
      if(task_res && curr_light_res)
      {
        std::cout << "task_res and curr_light there" << std::endl;
        float current_lighting = _light_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_FLOAT>().data;
        double num_objects_detected, detection_attempts;

        num_objects_detected = _deteff_metrics[0];
        detection_attempts = _deteff_metrics[1];
        double detection_ratio = num_objects_detected / detection_attempts;

        if((current_lighting < 0.70) || detection_ratio < 0.10)
        {
          return change_camera_feed(ALT_CAMERA_TOPIC);
        }
        return change_camera_feed(OG_CAMERA_TOPIC);  
      }
      else
      {
        std::cout << "task_res or curr_light not there" << std::endl;
      }

      return false;
    }
  private:
      static constexpr const char* IN_DETEFF = "deteff";
      static constexpr const char* IN_LIGHT = "lighting";

      std::string ALT_CAMERA_TOPIC = "/corner_camera/image_raw";
      std::string OG_CAMERA_TOPIC = "/head_front_camera/rgb/image_raw";

      rebet::SystemAttributeValue _light_attribute;
      std::vector<double> _deteff_metrics;

      std::string current_image_feed;
};

}   // namespace BT
