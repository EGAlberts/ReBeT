#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rebet_msgs/srv/request_adaptation.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rebet_msgs/msg/variable_parameters.hpp"
#include "rebet_msgs/msg/variable_parameter.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"

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
class AdaptNode : public DecoratorNode
{
public:

  static constexpr const char* VARIABLE_PARAMS = "variable_parameters";
  static constexpr const char* ADAP_STRAT = "adaptation_strategy";


  AdaptNode(const std::string& name, const NodeConfig& config) : DecoratorNode(name, config)
  {
  }

    static PortsList providedPorts()
    {
        PortsList ports = {OutputPort<VariableParameters>(VARIABLE_PARAMS, "What can be changed at runtime about this action"),
                           InputPort<std::string>(ADAP_STRAT, "Which strategy should be employed to decide on adaptations")
                          };
        return ports;
    }

  virtual ~AdaptNode() override = default;

  //Name for the weight input port
  static constexpr const char* WEIGHT = "weight";

  //Names for the metric output ports
  static constexpr const char* METRIC = "metric";
  static constexpr const char* MEAN_METRIC = "mean_metric";





private:
  int weight_;
  bool read_parameter_from_ports_;


  virtual NodeStatus tick() override
  {
    setStatus(NodeStatus::RUNNING);
    const NodeStatus child_status = child_node_->executeTick();

    switch (child_status)
    {
      case NodeStatus::SUCCESS: {
        resetChild();
        return NodeStatus::SUCCESS;
      }

      case NodeStatus::FAILURE: {
        resetChild();
        return NodeStatus::FAILURE;
      }

      case NodeStatus::RUNNING: {
        return NodeStatus::RUNNING;
      }

      case NodeStatus::SKIPPED: {
        return NodeStatus::SKIPPED;
      }
      case NodeStatus::IDLE: {
        throw LogicError("[", name(), "]: A child should not return IDLE");
      }
    }
    return status();

  }

  //void halt() override;

  protected:

    template <class T>
    void registerAdaptations(std::vector<T> param_values, std::string param_name, std::string server_name)
    {
        rebet_msgs::msg::VariableParameter variable_param = rebet_msgs::msg::VariableParameter();

        variable_param.name = param_name;
        variable_param.node_name = server_name;
        for (T val : param_values) {
            rclcpp::ParameterValue par_val = rclcpp::ParameterValue(val);
            variable_param.possible_values.push_back(par_val.to_value_msg());
        }
        _var_params.variable_parameters.push_back(variable_param); //vector of VariableParameter   
    }

    std::string adaptation_target_;

    VariableParameters _var_params = VariableParameters();

};

class OnStartAdapt : public AdaptNode
{
  public:
    OnStartAdapt(const std::string& name, const NodeConfig& config, const std::string& adaptation_target) : AdaptNode(name, config)
    { 
      std::cout << "\n\n\n\nSomeone created me a OnStartAdapt node!!!!\n\n\n\n\n" << std::endl;
      node_ = rclcpp::Node::make_shared("adapt_rq_client");

      adaptation_target_ = adaptation_target;
  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNode::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

  virtual NodeStatus tick() override
  {
  //Shamelessly taken from bt_service_node :) 
  if(status() == NodeStatus::IDLE )
  {
      callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
      adapt_client_ = node_->create_client<rebet_msgs::srv::RequestAdaptation>("/request_adaptation", rmw_qos_profile_services_default, callback_group_);

      setStatus(NodeStatus::RUNNING);

      response_received_ = false;
      future_response_ = {};
      // on_feedback_state_change_ = NodeStatus::RUNNING;
      response_ = {};

      std::string strategy_name;
      getInput(ADAP_STRAT,strategy_name);
      auto request = std::make_shared<rebet_msgs::srv::RequestAdaptation::Request>();

      request->adaptation_options = _var_params;
      request->task_identifier = registrationName();
      request->adaptation_strategy = strategy_name;
      request->adaptation_target = adaptation_target_;


      future_response_ = adapt_client_->async_send_request(request).share();
      time_request_sent_ = node_->now();

      return NodeStatus::RUNNING;
  }
  if (status() == NodeStatus::RUNNING)
  { 
    
    //std::cout << "Made it into running" << std::endl;
    callback_group_executor_.spin_some();
    
    // FIRST case: check if the goal request has a timeout
    if( !response_received_ )
    {
      std::cout << "no response received (yet)" << std::endl;

      auto const nodelay = std::chrono::milliseconds(0);
      auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

      auto ret = callback_group_executor_.spin_until_future_complete(future_response_, nodelay);

      if (ret != rclcpp::FutureReturnCode::SUCCESS)
      {
        std::cout << "Not a success response (yet)" << std::endl;

        if( (node_->now() - time_request_sent_) > timeout )
        {
          throw std::runtime_error("ran out of time trying to request adaptation, is your adaptation logic working properly?"); 
        }
        else{
          std::cout << "Not a success response (yet) returning running" << std::endl;

          return NodeStatus::RUNNING;
        }
      }
      else
      {
        std::cout << "Response received!" << std::endl;

        response_received_ = true;
        response_ = future_response_.get();
        std::cout << "Got response from future!" << std::endl;

        // future_response_ = {};

        //You could check response success here

        if (!response_) {
          throw std::runtime_error("Request was rejected by the service");
        }
      }
    }
    else
    {
      //Response received
      //std::cout << "Outside of IDLE within adapt dec node" << std::endl;
      const NodeStatus child_status = child_node_->executeTick();

      //std::cout << "ticked child in adapt dec" << std::endl;



      switch (child_status)
      {
        case NodeStatus::SUCCESS: {
          resetChild();
          std::cout << "success child in adapt dec" << std::endl;
          return NodeStatus::SUCCESS;
        }

        case NodeStatus::FAILURE: {
          resetChild();
          std::cout << "failure child in adapt dec" << std::endl;

          return NodeStatus::FAILURE;
        }

        case NodeStatus::RUNNING: {
          //std::cout << "running child in adapt dec" << std::endl;

          return NodeStatus::RUNNING;
        }
        case NodeStatus::SKIPPED: {
          std::cout << "skipped child in adapt dec" << std::endl;

          return NodeStatus::SKIPPED;
        }
        case NodeStatus::IDLE: {
          throw LogicError("[", name(), "]: A child should not return IDLE");
        }
      }
      //I don't know when this would happen but OK
      return status();

    }


  }
  //I don't know when this would happen but OK
  return NodeStatus::RUNNING;
  }




   

    private:
      rclcpp::CallbackGroup::SharedPtr callback_group_;
      rebet_msgs::srv::RequestAdaptation::Response::SharedPtr response_;
      std::chrono::milliseconds service_timeout_ = std::chrono::milliseconds(1000);
      bool response_received_ = false;
      rclcpp::Time time_request_sent_;
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Client<rebet_msgs::srv::RequestAdaptation>::SharedPtr adapt_client_;
      rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
      bool adapted_yet = false;
      std::shared_future<rebet_msgs::srv::RequestAdaptation::Response::SharedPtr> future_response_;
      // std::shared_future<rebet_msgs::srv::RequestAdaptation::Response::SharedPtr> adapt_result;
      // rclcpp::detail::FutureAndRequestId<std::future<SharedResponse>>
      // rclcpp::detail::FutureAndRequestId<std::future<rebet_msgs::srv::RequestAdaptation::Response::SharedPtr>> adapt_result;
      int _detected_in_window;
      builtin_interfaces::msg::Time _last_timestamp;
      double _max_detected;
      double _max_object_ps;
      int _window_length;
      int _window_start;
      int _counter;
      static constexpr const char* IN_OBJ = "objs_identified";

};

class OnRunningAdapt : public AdaptNode
{
  static constexpr const char* WINDOW_LEN = "adaptation_period";
  public:
    OnRunningAdapt(const std::string& name, const NodeConfig& config, const std::string& adaptation_target) : AdaptNode(name, config)
    { 
      std::cout << "\n\n\n\nSomeone created me a OnRunningAdapt node!!!!\n\n\n\n\n" << std::endl;
      node_ = rclcpp::Node::make_shared("adapt_rq_client");
      adaptation_target_ = adaptation_target;

    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNode::providedPorts();

      PortsList child_ports =  {
        InputPort<int>(WINDOW_LEN, "The periodicity with which the adaptation should occur during running.")
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

  virtual NodeStatus tick() override
  {
  //Shamelessly taken from bt_service_node :) 
  if(status() == NodeStatus::IDLE )
  {
      callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
      adapt_client_ = node_->create_client<rebet_msgs::srv::RequestAdaptation>("/request_adaptation", rmw_qos_profile_services_default, callback_group_);

      getInput(WINDOW_LEN,_window_length);

      setStatus(NodeStatus::RUNNING);

      response_received_ = false;

      return NodeStatus::RUNNING;
  }
  if (status() == NodeStatus::RUNNING)
  {

    auto curr_time_pointer = std::chrono::system_clock::now();
    int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
    int elapsed_seconds = current_time-_window_start;
    const NodeStatus child_status = child_node_->executeTick();


    bool window_expired = elapsed_seconds >= _window_length;
    // if(within_window){

    // }
    if(window_expired && !request_sent_){
      std::cout << "window expired no request sent" << std::endl;

      std::string strategy_name;
      getInput(ADAP_STRAT,strategy_name);
      
      auto request = std::make_shared<rebet_msgs::srv::RequestAdaptation::Request>();

      future_response_ = {};
      request->adaptation_options = _var_params;
      request->task_identifier = registrationName();
      request->adaptation_strategy = strategy_name;
      request->adaptation_target = adaptation_target_;
      
      future_response_ = adapt_client_->async_send_request(request).share();
      time_request_sent_ = node_->now();
      
      request_sent_ = true;
      response_received_ = false;
    } 
    else if(window_expired && request_sent_){
      
      callback_group_executor_.spin_some();

          // FIRST case: check if the goal request has a timeout
      if(!response_received_ )
      {
        std::cout << "no response received (yet)" << std::endl;

        auto const nodelay = std::chrono::milliseconds(0);
        auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

        auto ret = callback_group_executor_.spin_until_future_complete(future_response_, nodelay);

        if (ret != rclcpp::FutureReturnCode::SUCCESS)
        {
          std::cout << "Not a success response (yet)" << std::endl;

          if( (node_->now() - time_request_sent_) > timeout )
          {
            throw std::runtime_error("ran out of time trying to request adaptation, is your adaptation logic working properly?"); 
          }
          else{
            std::cout << "Not a success response (yet) returning running" << std::endl;
          }
        }
        else
        {
          std::cout << "Response received!" << std::endl;

          response_received_ = true;
          response_ = future_response_.get();
          std::cout << "Got response from future!" << std::endl;

          // future_response_ = {};
          //You could check response success here

          if (!response_) {
            throw std::runtime_error("Request was rejected by the service");
          }

          //Reset window
          _window_start = current_time;

          response_ = {};
          request_sent_ = false;
        }
      }

    }
    if( (elapsed_seconds % 3) == 0 ){
      std::cout << "window not expired yet " << elapsed_seconds << std::endl;
    }

    switch (child_status)
    {
      case NodeStatus::SUCCESS: {
        resetChild();
        std::cout << "success child in adapt dec" << std::endl;
        return NodeStatus::SUCCESS;
      }

      case NodeStatus::FAILURE: {
        resetChild();
        std::cout << "failure child in adapt dec" << std::endl;

        return NodeStatus::FAILURE;
      }

      case NodeStatus::RUNNING: {
        //std::cout << "running child in adapt dec" << std::endl;

        return NodeStatus::RUNNING;
      }
      case NodeStatus::SKIPPED: {
        std::cout << "skipped child in adapt dec" << std::endl;

        return NodeStatus::SKIPPED;
      }
      case NodeStatus::IDLE: {
        throw LogicError("[", name(), "]: A child should not return IDLE");
      }
    }
    //I don't know when this would happen but OK
    return status();

    
  }
  //I don't know when this would happen but OK
  return status();
  }




   

    private:
      rclcpp::CallbackGroup::SharedPtr callback_group_;
      rebet_msgs::srv::RequestAdaptation::Response::SharedPtr response_;
      std::chrono::milliseconds service_timeout_ = std::chrono::milliseconds(1000);
      bool response_received_ = false;
      rclcpp::Time time_request_sent_;
      bool request_sent_ = false;
      std::shared_ptr<rclcpp::Node> node_;
      rclcpp::Client<rebet_msgs::srv::RequestAdaptation>::SharedPtr adapt_client_;
      rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
      bool adapted_yet = false;
      std::shared_future<rebet_msgs::srv::RequestAdaptation::Response::SharedPtr> future_response_;
      // std::shared_future<rebet_msgs::srv::RequestAdaptation::Response::SharedPtr> adapt_result;
      // rclcpp::detail::FutureAndRequestId<std::future<SharedResponse>>
      // rclcpp::detail::FutureAndRequestId<std::future<rebet_msgs::srv::RequestAdaptation::Response::SharedPtr>> adapt_result;
      int _detected_in_window;
      builtin_interfaces::msg::Time _last_timestamp;
      double _max_detected;
      double _max_object_ps;
      int _window_length;
      int _window_start;
      int _counter;
      static constexpr const char* IN_OBJ = "objs_identified";

};

class AdaptPictureRate : public OnStartAdapt
{
  public:

    const std::string PICTURE_RT_PARAM = "pic_rate";
    const std::string ACTION_SRVR = "identify_action_server"; //now unnecessary, should be removed..

    AdaptPictureRate(const std::string& name, const NodeConfig& config) : OnStartAdapt(name, config, "blackboard")
    {
      std::vector<int> pc_rate_values{1, 3, 5, 7};
      registerAdaptations<int>(pc_rate_values, PICTURE_RT_PARAM, ACTION_SRVR);

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.
      setOutput(VARIABLE_PARAMS, _var_params); 
  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = OnStartAdapt::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }


};

class AdaptSpiralAltitude : public OnRunningAdapt
{
  public:

    const std::string ALTITUDE_PARAM = "spiral_altitude";
    const std::string ACTION_SRVR = "f_generate_search_path_node";

    AdaptSpiralAltitude(const std::string& name, const NodeConfig& config) : OnRunningAdapt(name, config, "ros_node")
    {
      std::vector<double> altitude_values{1.0, 2.0, 3.0};
      registerAdaptations<double>(altitude_values, ALTITUDE_PARAM, ACTION_SRVR);

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.
      setOutput(VARIABLE_PARAMS, _var_params); 
  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = OnRunningAdapt::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }


};

}   // namespace BT
