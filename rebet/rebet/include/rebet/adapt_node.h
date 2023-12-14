#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rebet_msgs/srv/online_adaptation.hpp"
#include "rebet_msgs/srv/offline_adaptation.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rebet_msgs/msg/adaptation.hpp"
#include "rebet_msgs/msg/adaptation_options.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
namespace BT
{
/**
 * @brief The _ is used to _ the _ it decorates.
 *
 *
 * 
 *
 * 
 * 
 * 
 * 
 * 

 */




class OfflineAdaptation {
  public:
    OfflineAdaptation() {}
};


enum class AdaptationType {Online, Offline};

enum class AdaptationTarget : int8_t {
  LifecycleTransition = 0, 
  RosParameter,
  BlackboardEntry};


class AdaptDecoratorBase {
protected:
    AdaptationTarget adaptation_target_;
    std::vector<rebet_msgs::msg::AdaptationOptions> _var_params = {};
    std::vector<lifecycle_msgs::msg::Transition> _transitions = {};
    std::shared_ptr<rclcpp::Node> node_;
    AdaptationType adaptation_type_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::vector<rebet_msgs::msg::Adaptation> _offline_adaptations;
    std::shared_future<rebet_msgs::srv::OnlineAdaptation::Response::SharedPtr> online_future_response_;
    std::shared_future<rebet_msgs::srv::OfflineAdaptation::Response::SharedPtr> offline_future_response_;
    rebet_msgs::srv::OnlineAdaptation::Response::SharedPtr online_response_;
    rebet_msgs::srv::OfflineAdaptation::Response::SharedPtr offline_response_;
    rclcpp::Client<rebet_msgs::srv::OnlineAdaptation>::SharedPtr online_adapt_client_;
    rclcpp::Client<rebet_msgs::srv::OfflineAdaptation>::SharedPtr offline_adapt_client_;





    static constexpr const char* ADAP_OPT = "adaptation_options";
    static constexpr const char* ADAP_STRAT = "adaptation_strategy";
    static constexpr const char* ADAP_SUB = "adaptation_subject";
    static constexpr const char* ADAP_LOC = "subject_location";

    template <typename AdaptationService, typename AdaptationRequest, typename AdaptationResponse>
    void sendAdaptationRequest(const std::string& service_name,
    const std::string& strategy_name,
    typename rclcpp::Client<AdaptationService>::SharedPtr& client,
    const std::string& registration_name)
    {
      client = node_->create_client<AdaptationService>(service_name, rmw_qos_profile_services_default, callback_group_);
      auto request = std::make_shared<AdaptationRequest>();

      if constexpr (std::is_same_v<AdaptationService, rebet_msgs::srv::OnlineAdaptation>)
      {
        request->adaptation_space = _var_params;
        request->task_identifier = registration_name;
        request->adaptation_strategy = strategy_name;
        online_future_response_ = client->async_send_request(request).share();

        
      }
      else if constexpr (std::is_same_v<AdaptationService, rebet_msgs::srv::OfflineAdaptation>)
      {
        request->adaptations = _offline_adaptations;
        offline_future_response_ = client->async_send_request(request).share();
      }
    }

  
};



template <class ParamT>
class AdaptNode : public DecoratorNode, public virtual AdaptDecoratorBase
{
public:
  
  AdaptNode(const std::string& name, const NodeConfig& config) : DecoratorNode(name, config)
  {
    node_ = rclcpp::Node::make_shared("adapt_rq_client" + name);
  }

    static PortsList providedPorts()
    {
        PortsList ports = {
                           InputPort<std::string>(ADAP_STRAT, "Which strategy should be employed to decide on adaptations"),
                           InputPort<std::string>(ADAP_SUB, "The name of the thing you adapt"),
                           InputPort<std::string>(ADAP_LOC, "Where the thing you adapt is located"),
                           InputPort<std::vector<ParamT>>(ADAP_OPT, "List of things to adapt with")
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


  protected:

    void registerAdaptations()
    {
        std::vector<ParamT> param_values;
        std::string param_name;
        std::string node_name;
        getInput(ADAP_OPT, param_values);
        getInput(ADAP_SUB, param_name);
        getInput(ADAP_LOC, node_name);

        rebet_msgs::msg::AdaptationOptions variable_param = rebet_msgs::msg::AdaptationOptions();
        // #name of the parameter
        // string name

        // #name of the node (if any) it belongs to
        // string node_name

        // #The type of adaptation being done to the target, this is constrained choice specified in Adaptation.msg
        // int8 adaptation_target_type
      
        // #A presumed finite set of acceptable values for the parameter to hold.
        // rcl_interfaces/ParameterValue[] possible_values 

        variable_param.name = param_name;
        variable_param.node_name = node_name;
        variable_param.adaptation_target_type = static_cast<int8_t>(adaptation_target_);
        for (ParamT val : param_values) {
            rclcpp::ParameterValue par_val = rclcpp::ParameterValue(val);
            variable_param.possible_values.push_back(par_val.to_value_msg());
        }
        _var_params.push_back(variable_param); //vector of VariableParameter   
    }
};

//Condition{OnStart,OnRunning,OnSuccess,OnFailure}

template <class ParamT>
class AdaptOnCondition : public AdaptNode<ParamT>, public virtual AdaptDecoratorBase
{
  public: 

    AdaptOnCondition(const std::string& name, const NodeConfig& config) : AdaptNode<ParamT>(name, config)
    { 
      std::cout << "\n\n\n\nSomeone created me a AdaptOnCondition node!!!!\n\n\n\n\n" << std::endl;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNode<ParamT>::providedPorts();

      PortsList child_ports =  {};
      child_ports.merge(base_ports);

      return child_ports;
    }

  protected:

    virtual bool evaluate_condition()
    {
      std::cout << "Here's where I would evaluate a condition... if I had one" << std::endl;
      return _condition_default;
    }

    bool _condition_default = false;
};


template <class ParamT>
class AdaptOnConditionOnStart : public AdaptOnCondition<ParamT>, public virtual AdaptDecoratorBase
{
  public:
    AdaptOnConditionOnStart(const std::string& name, const NodeConfig& config, const AdaptationTarget& adaptation_target, const AdaptationType& adaptation_type) : AdaptOnCondition<ParamT>(name, config)
    { 
      std::cout << "\n\n\n\nSomeone created me a OnStartAdapt node!!!!\n\n\n\n\n" << std::endl;
      adaptation_target_ = adaptation_target;
      
      adaptation_type_ = adaptation_type;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnCondition<ParamT>::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

  virtual NodeStatus tick() override
  {
  //Shamelessly taken from bt_service_node :) 
  if(this->status() == NodeStatus::IDLE )
  {
      _condition_was_true = this->evaluate_condition();
      
      if(_condition_was_true)
      {
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

        this->setStatus(NodeStatus::RUNNING);

        response_received_ = false;

        std::string strategy_name;
        this->getInput(ADAP_STRAT,strategy_name);

        if(adaptation_type_ == AdaptationType::Online)
        {
          sendAdaptationRequest<rebet_msgs::srv::OnlineAdaptation, rebet_msgs::srv::OnlineAdaptation::Request, rebet_msgs::srv::OnlineAdaptation::Response>(
            "/online_adaptation", strategy_name, online_adapt_client_, this->registrationName());
            
        }
        else if(adaptation_type_ == AdaptationType::Offline)
        {
          sendAdaptationRequest<rebet_msgs::srv::OfflineAdaptation, rebet_msgs::srv::OfflineAdaptation::Request, rebet_msgs::srv::OfflineAdaptation::Response>(
            "/offline_adaptation", strategy_name, offline_adapt_client_, this->registrationName());
        }
        time_request_sent_ = node_->now();
      }
      else{
        response_received_ = true; //Skipping over the response waiting logic.
      }

      return NodeStatus::RUNNING;
  }
  if (this->status() == NodeStatus::RUNNING)
  { 
      // FIRST case: check if the goal request has a timeout
      if( !response_received_ )
      {
        callback_group_executor_.spin_some();

        // std::cout << "no response received (yet)" << std::endl;

        auto const nodelay = std::chrono::milliseconds(0);
        auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

        rclcpp::FutureReturnCode ret;

        if(adaptation_type_ == AdaptationType::Online){
          ret = callback_group_executor_.spin_until_future_complete(online_future_response_, nodelay);
        }
        else if(adaptation_type_ == AdaptationType::Offline){
          ret = callback_group_executor_.spin_until_future_complete(offline_future_response_, nodelay);
        }

        if (ret != rclcpp::FutureReturnCode::SUCCESS)
        {
          if( (node_->now() - time_request_sent_) > timeout )
          {
            throw std::runtime_error("ran out of time trying to request adaptation, is your adaptation logic working properly?"); 
          }
          else{
            return NodeStatus::RUNNING;
          }
        }
        else
        {
          response_received_ = true;

          if(adaptation_type_ == AdaptationType::Online){
            std::cout << "Response received!" << std::endl;

            online_response_ = online_future_response_.get();
            std::cout << "Got response from future!" << std::endl;

            // future_response_ = {};

            //You could check response success here

            if (!online_response_) {
              throw std::runtime_error("Request was rejected by the service");
            }
          }
          else if(adaptation_type_ == AdaptationType::Offline){
            std::cout << "Response received!" << std::endl;

            offline_response_ = offline_future_response_.get();
            std::cout << "Got response from future!" << std::endl;

            // future_response_ = {};

            //You could check response success here

            if (!offline_response_) {
              throw std::runtime_error("Request was rejected by the service");
            }
          }
         
        }
      }

    else
    {
      //Response received
      //std::cout << "Outside of IDLE within adapt dec node" << std::endl;
      const NodeStatus child_status = this->child_node_->executeTick();

      //std::cout << "ticked child in adapt dec" << std::endl;



      switch (child_status)
      {
        case NodeStatus::SUCCESS: {
          this->resetChild();
          std::cout << "success child in adapt dec" << std::endl;
          return NodeStatus::SUCCESS;
        }

        case NodeStatus::FAILURE: {
          this->resetChild();
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
          throw LogicError("[", this->name(), "]: A child should not return IDLE");
        }
      }
      //I don't know when this would happen but OK
      return this->status();

    }


  }
  //I don't know when this would happen but OK
  return NodeStatus::RUNNING;
  }




   

  protected:
    rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
    bool _condition_was_true;
    std::chrono::milliseconds service_timeout_ = std::chrono::milliseconds(1000);
    bool response_received_ = false;
    rclcpp::Time time_request_sent_;

  private:







    bool adapted_yet = false;

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
};

//It is not lost on me that right now adapt on running is just oncondition with a time window
template <class ParamT>
class AdaptOnConditionOnRunning : public AdaptOnCondition<ParamT>, public virtual AdaptDecoratorBase
{
  public:
    AdaptOnConditionOnRunning(const std::string& name, const NodeConfig& config, const AdaptationTarget& adaptation_target, AdaptationType adaptation_type) : AdaptOnCondition<ParamT>(name, config)
    { 
      std::cout << "\n\n\n\nSomeone created me a AdaptOnConditionOnRunning node!!!!\n\n\n\n\n" << std::endl;
      adaptation_target_ = adaptation_target;
      adaptation_type_ = adaptation_type;

    }
    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnCondition<ParamT>::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }



  virtual NodeStatus tick() override
  {
  //Shamelessly taken from bt_service_node :) 
  if(this->status() == NodeStatus::IDLE )
  {
      callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
      
      this->setStatus(NodeStatus::RUNNING);

      response_received_ = false;

      return NodeStatus::RUNNING;
  }
  if (this->status() == NodeStatus::RUNNING)
  {
    const NodeStatus child_status = this->child_node_->executeTick();

    if(!_to_adapt) //I only want to evaluate the again condition after a trigger is processed.
    {
      _to_adapt = this->evaluate_condition();
    }

    if(_to_adapt && !request_sent_){
      std::cout << "condition met but no request sent" << std::endl;

      std::string strategy_name;
      this->getInput(ADAP_STRAT,strategy_name);
      
      // auto request = std::make_shared<rebet_msgs::srv::RequestAdaptation::Request>();

      // future_response_ = {};
      // request->adaptation_options = _var_params;
      // request->task_identifier = this->registrationName();
      // request->adaptation_strategy = strategy_name;
      // request->adaptation_target = adaptation_target_;
      // request->lifecycle_transitions = _transitions;

      
      // future_response_ = adapt_client_->async_send_request(request).share();

        if(adaptation_type_ == AdaptationType::Online)
        {
          sendAdaptationRequest<rebet_msgs::srv::OnlineAdaptation, rebet_msgs::srv::OnlineAdaptation::Request, rebet_msgs::srv::OnlineAdaptation::Response>(
            "/online_adaptation", strategy_name, online_adapt_client_, this->registrationName());
            
        }
        else if(adaptation_type_ == AdaptationType::Offline)
        {
          sendAdaptationRequest<rebet_msgs::srv::OfflineAdaptation, rebet_msgs::srv::OfflineAdaptation::Request, rebet_msgs::srv::OfflineAdaptation::Response>(
            "/offline_adaptation", strategy_name, offline_adapt_client_, this->registrationName());
        }
      time_request_sent_ = node_->now();
      
      request_sent_ = true;
      response_received_ = false;
    } 
    else if(_to_adapt && request_sent_){
      

      // FIRST case: check if the goal request has a timeout
      if(!response_received_ )
      {
        // std::cout << "no response received (yet)" << std::endl;
        callback_group_executor_.spin_some();


        auto const nodelay = std::chrono::milliseconds(0);
        auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

        rclcpp::FutureReturnCode ret;

        if(adaptation_type_ == AdaptationType::Online){
          ret = callback_group_executor_.spin_until_future_complete(online_future_response_, nodelay);
        }
        else if(adaptation_type_ == AdaptationType::Offline){
          ret = callback_group_executor_.spin_until_future_complete(offline_future_response_, nodelay);
        }

        if (ret != rclcpp::FutureReturnCode::SUCCESS)
        {
          if( (node_->now() - time_request_sent_) > timeout )
          {
            throw std::runtime_error("ran out of time trying to request adaptation, is your adaptation logic working properly?"); 
          }
          else{
            return NodeStatus::RUNNING;
          }
        }
        else
        {
          std::cout << "Response received!" << std::endl;

          response_received_ = true;
          if(adaptation_type_ == AdaptationType::Online){
            std::cout << "Response received!" << std::endl;

            online_response_ = online_future_response_.get();
            std::cout << "Got response from future!" << std::endl;

            // future_response_ = {};

            //You could check response success here

            if (!online_response_) {
              throw std::runtime_error("Request was rejected by the service");
            }
          }
          else if(adaptation_type_ == AdaptationType::Offline){
            std::cout << "Response received!" << std::endl;

            offline_response_ = offline_future_response_.get();
            std::cout << "Got response from future!" << std::endl;

            // future_response_ = {};

            //You could check response success here

            if (!offline_response_) {
              throw std::runtime_error("Request was rejected by the service");
            }

          }

          _to_adapt = false; //Reset condition.

          offline_response_ = {};
          online_response_ = {};
          request_sent_ = false;
        }
      }

    }
    // if( (elapsed_seconds % 3) == 0 ){
    //   std::cout << "window not expired yet " << elapsed_seconds << std::endl;
    // }

    switch (child_status)
    {
      case NodeStatus::SUCCESS: {
        this->resetChild();
        std::cout << "success child in adapt dec" << std::endl;
        return NodeStatus::SUCCESS;
      }

      case NodeStatus::FAILURE: {
        this->resetChild();
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
        throw LogicError("[", this->name(), "]: A child should not return IDLE");
      }
    }
    //I don't know when this would happen but OK
    return this->status();

    
  }
  //I don't know when this would happen but OK
  return this->status();
  }

    private:
      std::chrono::milliseconds service_timeout_ = std::chrono::milliseconds(1000);
      bool response_received_ = false;
      rclcpp::Time time_request_sent_;
      bool request_sent_ = false;
      rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
      bool adapted_yet = false;
      bool _to_adapt = false;
      builtin_interfaces::msg::Time _last_timestamp;
      

      int _counter;
    protected:
      bool _condition_default = false;

};

template <class ParamT>
class AdaptOnConditionOnSuccess : public AdaptOnCondition<ParamT>, public virtual AdaptDecoratorBase
{
  public:
    AdaptOnConditionOnSuccess(const std::string& name, const NodeConfig& config, const AdaptationTarget& adaptation_target, AdaptationType adaptation_type) : AdaptOnCondition<ParamT>(name, config)
    { 
      std::cout << "\n\n\n\nSomeone created me a AdaptOnConditionOnSuccess node!!!!\n\n\n\n\n" << std::endl;
      adaptation_target_ = adaptation_target;
      adaptation_type_ = adaptation_type;

    }
    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnCondition<ParamT>::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }



  virtual NodeStatus tick() override
  {
  //Shamelessly taken from bt_service_node :) 
  if(this->status() == NodeStatus::IDLE )
  {
      callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
      
      this->setStatus(NodeStatus::RUNNING);

      response_received_ = false;

      return NodeStatus::RUNNING;
  }
  if (this->status() == NodeStatus::RUNNING)
  {
    if(!_condition_was_true)
    {
      child_status_ = this->child_node_->executeTick();
    }

    switch (child_status_)
    {
      case NodeStatus::SUCCESS: {
        if(!_condition_was_true)
        {
          _condition_was_true = this->evaluate_condition();
        }
        
        if(_condition_was_true && !request_sent_)
        {
          callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    
          response_received_ = false;

          std::string strategy_name;
          this->getInput(ADAP_STRAT,strategy_name);

        if(adaptation_type_ == AdaptationType::Online)
        {
          sendAdaptationRequest<rebet_msgs::srv::OnlineAdaptation, rebet_msgs::srv::OnlineAdaptation::Request, rebet_msgs::srv::OnlineAdaptation::Response>(
            "/online_adaptation", strategy_name, online_adapt_client_, this->registrationName());
            
        }
        else if(adaptation_type_ == AdaptationType::Offline)
        {
          sendAdaptationRequest<rebet_msgs::srv::OfflineAdaptation, rebet_msgs::srv::OfflineAdaptation::Request, rebet_msgs::srv::OfflineAdaptation::Response>(
            "/offline_adaptation", strategy_name, offline_adapt_client_, this->registrationName());
        }
          time_request_sent_ = node_->now();
          request_sent_ = true;
        }
        else if(_condition_was_true && request_sent_)
        {
          // FIRST case: check if the goal request has a timeout
          if(!response_received_ )
          {
            // std::cout << "no response received (yet)" << std::endl;
            callback_group_executor_.spin_some();


            auto const nodelay = std::chrono::milliseconds(0);
            auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

            rclcpp::FutureReturnCode ret;

            if(adaptation_type_ == AdaptationType::Online){
              ret = callback_group_executor_.spin_until_future_complete(online_future_response_, nodelay);
            }
            else if(adaptation_type_ == AdaptationType::Offline){
              ret = callback_group_executor_.spin_until_future_complete(offline_future_response_, nodelay);
            }

            if (ret != rclcpp::FutureReturnCode::SUCCESS)
            {
              if( (node_->now() - time_request_sent_) > timeout )
              {
                throw std::runtime_error("ran out of time trying to request adaptation, is your adaptation logic working properly?"); 
              }
              else{
                return NodeStatus::RUNNING;
              }
            }
            else
            {
              std::cout << "Response received!" << std::endl;

              response_received_ = true;
              if(adaptation_type_ == AdaptationType::Online){
                std::cout << "Response received!" << std::endl;

                online_response_ = online_future_response_.get();
                std::cout << "Got response from future!" << std::endl;

                // future_response_ = {};

                //You could check response success here

                if (!online_response_) {
                  throw std::runtime_error("Request was rejected by the service");
                }
              }
              else if(adaptation_type_ == AdaptationType::Offline){
                std::cout << "Response received!" << std::endl;

                offline_response_ = offline_future_response_.get();
                std::cout << "Got response from future!" << std::endl;

                // future_response_ = {};

                //You could check response success here

                if (!offline_response_) {
                  throw std::runtime_error("Request was rejected by the service");
                }

              }

              _to_adapt = false; //Reset condition.

              offline_response_ = {};
              online_response_ = {};
              request_sent_ = false;
            }
          }

        }

        if(response_received_ || !_condition_was_true)
        {
          this->resetChild();
          std::cout << "success child in adapt dec" << std::endl;
          return NodeStatus::SUCCESS;
        }

      }

      case NodeStatus::FAILURE: {
        this->resetChild();
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
        throw LogicError("[", this->name(), "]: A child should not return IDLE");
      }
    }
    //I don't know when this would happen but OK
    return this->status();

    
  }
  //I don't know when this would happen but OK
  return this->status();
  }

    private:
      std::chrono::milliseconds service_timeout_ = std::chrono::milliseconds(1000);
      bool response_received_ = false;
      rclcpp::Time time_request_sent_;
      bool request_sent_ = false;
      bool _condition_was_true = false;
      rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
      bool adapted_yet = false;
      bool _to_adapt = false;
      builtin_interfaces::msg::Time _last_timestamp;
      NodeStatus child_status_;
      

      int _counter;
    protected:
      bool _condition_default = false;

};

template <class ParamT>
class AdaptOnConditionOnFailure : public AdaptOnCondition<ParamT>, public virtual AdaptDecoratorBase
{
  public:
    AdaptOnConditionOnFailure(const std::string& name, const NodeConfig& config, const AdaptationTarget& adaptation_target, AdaptationType adaptation_type) : AdaptOnCondition<ParamT>(name, config)
    { 
      std::cout << "\n\n\n\nSomeone created me a AdaptOnConditionOnSuccess node!!!!\n\n\n\n\n" << std::endl;
      adaptation_target_ = adaptation_target;
      adaptation_type_ = adaptation_type;

    }
    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnCondition<ParamT>::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }



  virtual NodeStatus tick() override
  {
  //Shamelessly taken from bt_service_node :) 
  if(this->status() == NodeStatus::IDLE )
  {
      callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
      
      this->setStatus(NodeStatus::RUNNING);

      response_received_ = false;

      return NodeStatus::RUNNING;
  }
  if (this->status() == NodeStatus::RUNNING)
  {
    if(!_condition_was_true)
    {
      child_status_ = this->child_node_->executeTick();
    }

    switch (child_status_)
    {
      case NodeStatus::SUCCESS: {
          this->resetChild();
          std::cout << "success child in adapt dec" << std::endl;
          return NodeStatus::SUCCESS;
      }

      case NodeStatus::FAILURE: {
        if(!_condition_was_true)
        {
          _condition_was_true = this->evaluate_condition();
        }
        
        if(_condition_was_true && !request_sent_)
        {
          callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
          callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    
          response_received_ = false;

          std::string strategy_name;
          this->getInput(ADAP_STRAT,strategy_name);

          if(adaptation_type_ == AdaptationType::Online)
          {
            sendAdaptationRequest<rebet_msgs::srv::OnlineAdaptation, rebet_msgs::srv::OnlineAdaptation::Request, rebet_msgs::srv::OnlineAdaptation::Response>(
              "/online_adaptation", strategy_name, online_adapt_client_, this->registrationName());
              
          }
          else if(adaptation_type_ == AdaptationType::Offline)
          {
            sendAdaptationRequest<rebet_msgs::srv::OfflineAdaptation, rebet_msgs::srv::OfflineAdaptation::Request, rebet_msgs::srv::OfflineAdaptation::Response>(
              "/offline_adaptation", strategy_name, offline_adapt_client_, this->registrationName());
          }
          time_request_sent_ = node_->now();
          request_sent_ = true;
        }
        else if(_condition_was_true && request_sent_)
        {
          // FIRST case: check if the goal request has a timeout
          if(!response_received_ )
          {
            // std::cout << "no response received (yet)" << std::endl;
            callback_group_executor_.spin_some();


            auto const nodelay = std::chrono::milliseconds(0);
            auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

            rclcpp::FutureReturnCode ret;

            if(adaptation_type_ == AdaptationType::Online){
              ret = callback_group_executor_.spin_until_future_complete(online_future_response_, nodelay);
            }
            else if(adaptation_type_ == AdaptationType::Offline){
              ret = callback_group_executor_.spin_until_future_complete(offline_future_response_, nodelay);
            }


            if (ret != rclcpp::FutureReturnCode::SUCCESS)
            {
              // std::cout << "Not a success response (yet)" << std::endl;

              if( (node_->now() - time_request_sent_) > timeout )
              {
                throw std::runtime_error("ran out of time trying to request adaptation, is your adaptation logic working properly?"); 
              }
            }
            else
            {
              std::cout << "Response received!" << std::endl;

              response_received_ = true;
              if(adaptation_type_ == AdaptationType::Online){
                std::cout << "Response received!" << std::endl;

                online_response_ = online_future_response_.get();
                std::cout << "Got response from future!" << std::endl;

                // future_response_ = {};

                //You could check response success here

                if (!online_response_) {
                  throw std::runtime_error("Request was rejected by the service");
                }
              }
              else if(adaptation_type_ == AdaptationType::Offline){
                std::cout << "Response received!" << std::endl;

                offline_response_ = offline_future_response_.get();
                std::cout << "Got response from future!" << std::endl;

                // future_response_ = {};

                //You could check response success here

                if (!offline_response_) {
                  throw std::runtime_error("Request was rejected by the service");
                }

              }

              _to_adapt = false; //Reset condition.

              offline_response_ = {};
              online_response_ = {};
              request_sent_ = false;
          }

        }

        if(response_received_ || !_condition_was_true)
        {
          this->resetChild();
          std::cout << "success child in adapt dec" << std::endl;
          return NodeStatus::FAILURE;
        }
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
        throw LogicError("[", this->name(), "]: A child should not return IDLE");
      }
    }
    //I don't know when this would happen but OK
    return this->status();

    
  }
  //I don't know when this would happen but OK
  return this->status();
  }
  }

    private:
      std::chrono::milliseconds service_timeout_ = std::chrono::milliseconds(1000);
      bool response_received_ = false;
      rclcpp::Time time_request_sent_;
      bool request_sent_ = false;
      bool _condition_was_true = false;
      rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
      bool adapted_yet = false;
      bool _to_adapt = false;
      builtin_interfaces::msg::Time _last_timestamp;
      NodeStatus child_status_;
      

      int _counter;
    protected:
      bool _condition_default = false;

};


template <class ParamT>
class AdaptPeriodicallyOnRunning : public AdaptOnConditionOnRunning<ParamT>, public virtual AdaptDecoratorBase
{
  static constexpr const char* WINDOW_LEN = "adaptation_period";
  public:
    AdaptPeriodicallyOnRunning(const std::string& name, const NodeConfig& config, const AdaptationTarget& adaptation_target, AdaptationType adaptation_type) : AdaptOnConditionOnRunning<ParamT>(name, config, adaptation_target, adaptation_type)
    { 
      std::cout << "\n\n\n\nSomeone created me a AdaptPeriodicallyOnRunning node!!!!\n\n\n\n\n" << std::endl;
      auto curr_time_pointer = std::chrono::system_clock::now();
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
     
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptOnConditionOnRunning<ParamT>::providedPorts();

      PortsList child_ports =  {
        InputPort<int>(WINDOW_LEN, "The periodicity with which the adaptation should occur during running.")
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual bool evaluate_condition() override
    {
      this->getInput(WINDOW_LEN,_window_length);
      auto curr_time_pointer = std::chrono::system_clock::now();
      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;

      bool window_expired = elapsed_seconds >= _window_length;

      if(window_expired)
      {
        _window_start = current_time;
      }
      return window_expired;
    }

    private:
      int _window_length;
      int _window_start;
};






class AdaptSpiralAltitudeOnline : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptSpiralAltitudeOnline(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::Online)
    {
      registerAdaptations();

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
