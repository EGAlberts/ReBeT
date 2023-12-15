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

#include "rebet_frog/frog_constants.hpp"


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

class AdaptPictureRateOffline: public AdaptOnConditionOnStart<int>
{
  public:

    AdaptPictureRateOffline(const std::string& name, const NodeConfig& config) : AdaptOnConditionOnStart<int>(name, config, AdaptationTarget::BlackboardEntry, AdaptationType::Offline)
    {
      _condition_default = true;
      registerAdaptations();
      _pictures_taken = 0;
      _power_consumed = 0.0;

      
      

      // auto resetCallback = [this](TimePoint timestamp, const TreeNode& node,
      //                                   NodeStatus prev, NodeStatus status) {
      //     std::cout << "\n\n STATUS CHANGED CALLBACK CALLED STATUS IS " << status << std::endl;
      //     if ((status == NodeStatus::IDLE))
      //     {

      //       this->_pictures_taken = 0;
      //       this->_power_consumed = 0.0;
      //       this->_obs_taken_pictures_of = 0;
      //     }
      //   };
      // this->subscribeToStatusChange(std::move(resetCallback));

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.  
    }

    void resetTracking()
    {
        std::cout << "\n\n\n RRESET TRACKING \n\n\n" << std::endl;
      _pictures_taken = 0;
      _power_consumed = 0.0;
      _obs_taken_pictures_of = 0;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnStart::providedPorts();

      PortsList child_ports =  {
        InputPort<double>(POW_IN,"the power metric"),
        InputPort<double>(PICTASK_IN,"the det obj task metric"),
        InputPort<std::string>(TASK_STATE,"state of the det obj task metric"),
        InputPort<int>(TOT_OBS, "how many obstacles there are"),
        OutputPort<int>(OUT_PIC, "elastic picture rate")
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    void pic_adaptation_msg(int pic_rate)
    {

      std::cout << "pic rate being put into adaptation msg " << pic_rate << std::endl; 
      setOutput(OUT_PIC, pic_rate);
      std::string param_name;

      _current_pic_rate = pic_rate;
      getInput(ADAP_SUB, param_name);

      rebet_msgs::msg::Adaptation adap;
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(pic_rate));

      std::cout << "type name " << adap_param.get_type_name() << std::endl;

      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::BlackboardEntry);
      adap.blackboard_adaptation = adap_param.to_parameter_msg();

      _offline_adaptations.push_back(adap);

    }

    bool decrease_pic_rate()
    {
      _offline_adaptations = {};

      int new_pic_rate = _current_pic_rate - PIC_INCREMENT;

      if(new_pic_rate < MIN_PIC_INCREMENT)
      {
        std::cout << " pic rate decrease not possible " << std::endl;

        return false;
      }

      pic_adaptation_msg(new_pic_rate);

      return true;
    }

    void increase_pic_rate()
    {
      _offline_adaptations = {};

      int new_pic_rate = _current_pic_rate + PIC_INCREMENT;

      

      pic_adaptation_msg(new_pic_rate);

    }

    void set_pic_rate(int new_pic_rate)
    {
      _offline_adaptations = {};

      pic_adaptation_msg(new_pic_rate);

    }

    virtual bool evaluate_condition() override
    {
      //By virtue of being called, it means a picture has been taken
      _pictures_taken+=1;

      double current_power;
      getInput(POW_IN, current_power);

      //The power consumed for said picture.
      _power_consumed += current_power;

      //Was an object detected in that picture?
      std::string state_of_task;
      getInput(TASK_STATE,state_of_task);

      std::cout << "\n\n\n state of task" << state_of_task << "\n\n" << std::endl;
      objs_detected.push(state_of_task);

      std::cout << "obs taken pics of " << _obs_taken_pictures_of << std::endl;



      if((_pictures_taken-1) % PIC_INCREMENT == 0 && _pictures_taken != 1) //Was that picture the nth picture?
      {
        std::cout << "\n\n\n it was the nth picture \n\n\n" << std::endl;
        //Whether or not to take another PIC_INCREMENT pics.
        auto it = std::find(objs_detected.getContainer().begin(), objs_detected.getContainer().end(), detected);

        int obs_num;
        getInput(TOT_OBS, obs_num);
        //If there was an object in any of the PIC_INCREMENT number of pictures taken
        if (it != objs_detected.getContainer().end())
        {

          std::cout << "\n\n An object was detected \n\n" << std::endl;

          //Potentially take some more pictures.


          //As long as it remains possible to take 6 more pictures of the remaining obstacles before exceeding the budget.


          std::cout << " obs num " << obs_num << std::endl;
          int obs_remaining = obs_num - _obs_taken_pictures_of;

          double power_needed_after = (double)obs_remaining * (double)(2 * (PIC_INCREMENT+1)) * DETECTION_AVG_POW; //Power still necessary

          double total_power_budget = (double)obs_num * (double)(2 * (PIC_INCREMENT+1)) * DETECTION_AVG_POW;

          double power_needed_for_extra = PIC_INCREMENT * DETECTION_AVG_POW;

          //Is there at least the power, 
          double power_left = total_power_budget - _power_consumed;
          //to take PIC_INCREMENT extra pictures while also taking PIC_INCREMENT + 1 pics of the rest?

          std::cout << "\n\n  pow left " << power_left << " power_needed_after " << power_needed_after <<  " power_needed for extra " << power_needed_for_extra  << std::endl;

          if(power_left >= (power_needed_after + power_needed_for_extra) && power_left > 0.0) 
          {
            std::cout << "increase pic rate" << std::endl;
            increase_pic_rate();
            return true;
            //Its OK to take more pictures

          }
          else
          {
            std::cout << "no increase pic rate" << std::endl;

            _pictures_taken = 0;
            _obs_taken_pictures_of+=1;

            if(_obs_taken_pictures_of == obs_num)
            {
              resetTracking();
            }

            return false;
            //I'm done with this obstacle

          }


        }
        else
        {

          //No more pictures.
          std::cout << "stop pic rate" << std::endl;


          //This sets the rate back if it comes after an extended session.
          set_pic_rate(_pictures_taken); //Till here and no further.

          _pictures_taken = 0;
          _obs_taken_pictures_of+=1;

          if(_obs_taken_pictures_of == obs_num)
          {
            resetTracking();
          }


          return true;
        }
            
      }
      else
      {
        // _obs_taken_pictures_of+=1;
        return false;
      }

      return false;
    }
  private:
      static constexpr const char* POW_IN = "in_power";
      static constexpr const char* PICTASK_IN = "in_pictask";
      static constexpr const char* TASK_STATE = "in_pictask_state";
      static constexpr const char* TOT_OBS = "obstacles_total";
      static constexpr const char* OUT_PIC = "out_pic_rate";

      std::string detected = std::string(OBJECT_DETECTED_STRING);

      static constexpr int PIC_INCREMENT = 2;
      const int MIN_PIC_INCREMENT = 1;
      const int MAX_PIC_INCREMENT = 7;
      int _current_pic_rate = 5;

      int _pictures_taken;
      double _power_consumed;

      int _obs_taken_pictures_of = 0;
      FixedQueue<std::string, PIC_INCREMENT> objs_detected;



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
        std::cout << "\n\n\n\n\n\n\n\n\n\n checking for charge " << sum_power_metric_observed << "\n\n\n\n\n\n\n\n\n\n checking for charge " << std::endl;
        sum_power_metric_observed+=1.0; //curr_power_metric; Just going to count a number of times for now. You can see how easily this could use a real power value.
        if(sum_power_metric_observed > CHARGE_THRESHOLD) //
        {
          RCLCPP_INFO(node_->get_logger(), "\n\n\nPower Tresh Exceed!\n\n\n");
          sum_power_metric_observed = 0.0;

          std::string param_name;
          getInput(ADAP_SUB, param_name);
          rebet_msgs::msg::Adaptation adap;
          rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue("charge"));
          adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::BlackboardEntry);
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
    const double CHARGE_THRESHOLD = 2.0;
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


class AdaptMaxSpeedOffline : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptMaxSpeedOffline(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::Offline)
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

    void speed_adaptation_msg(double max_speed_value)
    {
      _current_max_speed = max_speed_value;

      std::vector<double> speed_vector = {max_speed_value,_default_y_velocity,_default_theta_velocity};

      std::string param_name;
      std::string node_name;


      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);


      rebet_msgs::msg::Adaptation adap;
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(speed_vector));
      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _offline_adaptations.push_back(adap);

    }

    bool decrease_speed()
    {
      _offline_adaptations = {};

      double new_max_speed = _current_max_speed - SPEED_INCREMENT;

      if(new_max_speed < MIN_SPEED)
      {
        std::cout << "no decrease speed possible " << std::endl;

        return false;
      }

      speed_adaptation_msg(new_max_speed);

      return true;
    }

    bool increase_speed()
    {
      _offline_adaptations = {};

      double new_max_speed = _current_max_speed + SPEED_INCREMENT;

      if(new_max_speed > MAX_MAX_SPEED)
      {
        return false;
      }

      speed_adaptation_msg(new_max_speed);

      return true;
    }

    virtual bool evaluate_condition() override
    {
      if(AdaptPeriodicallyOnRunning::evaluate_condition())
      {
        double current_safety;
        double current_power;
        double current_move;
        auto safe_res = getInput(SAFE_IN, current_safety);
        auto pow_res = getInput(POW_IN, current_power);
        auto move_res = getInput(MOVE_IN, current_move);





        std::cout << "I'm here in offline max spd! \n\n" << std::endl;
        if(safe_res && pow_res && move_res)
        {
          std::cout << "now here curr_safe " << current_safety << " curr_pow " << current_power << std::endl;

          if( (current_safety < 0.09) || (current_power > 5.0) )
          {
            std::cout << "decrease speed here " << std::endl;
            

            return decrease_speed();
          }
          if(current_power < 4.0 || current_safety > 0.15 || current_move < 0.40)
          {
            std::cout << "increase speed here " << std::endl;

            return increase_speed();
          }
        }
      }
      return false;    
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {
        InputPort<double>(POW_IN,"the power metric"),
        InputPort<double>(SAFE_IN,"the safety metric"),
        InputPort<double>(MOVE_IN,"the movement efficiency"),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

  private:
    double _default_y_velocity = 0.0; //The robot in question can not move along its y axis independently.
    double _default_theta_velocity = 2.5; //We are not interested in modifying the rotation speed here, but could be extended to do so.
    static constexpr const char* POW_IN = "in_power";
    static constexpr const char* SAFE_IN = "in_safety";
    static constexpr const char* MOVE_IN = "in_movement";
    const double SPEED_INCREMENT = 0.08;
    const double MIN_SPEED = 0.08;
    const double MAX_MAX_SPEED = 0.26;
    double _current_max_speed = MAX_MAX_SPEED;
};



}   // namespace BT
