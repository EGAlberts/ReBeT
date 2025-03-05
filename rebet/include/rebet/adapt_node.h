#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "rclcpp/rclcpp.hpp"

#include "aal_msgs/srv/adapt_architecture_external.hpp"
#include "aal_msgs/srv/adapt_architecture.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "aal_msgs/msg/adaptation.hpp"
#include "aal_msgs/msg/adaptation_options.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

namespace BT
{
/**
 * @brief The AdaptDecorator is used to architecturally adapt the ROS2 nodes corresponding to BT nodes it decorates.
 */


enum class AdaptationType {External, Internal};

enum class AdaptationTarget : int8_t {
  LifecycleTransition = 0, 
  RosParameter};


class AdaptDecoratorBase {
protected:
    AdaptationTarget adaptation_target_;
    std::vector<aal_msgs::msg::AdaptationOptions> _var_params = {};
    std::vector<lifecycle_msgs::msg::Transition> _transitions = {};
    std::shared_ptr<rclcpp::Node> node_;
    AdaptationType adaptation_type_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::vector<aal_msgs::msg::Adaptation> _internal_adaptations;
    std::shared_future<aal_msgs::srv::AdaptArchitectureExternal::Response::SharedPtr> external_future_response_;
    std::shared_future<aal_msgs::srv::AdaptArchitecture::Response::SharedPtr> internal_future_response_;
    aal_msgs::srv::AdaptArchitectureExternal::Response::SharedPtr external_response_;
    aal_msgs::srv::AdaptArchitecture::Response::SharedPtr internal_response_;
    rclcpp::Client<aal_msgs::srv::AdaptArchitectureExternal>::SharedPtr external_adapt_client_;
    rclcpp::Client<aal_msgs::srv::AdaptArchitecture>::SharedPtr internal_adapt_client_;
    std::vector<double> _current_utilities = {};
    bool response_received_ = false;
    std::chrono::milliseconds service_timeout_ = std::chrono::milliseconds(ADAP_SERVICE_TIMEOUT_MILLISECOND);
    rclcpp::Time time_request_sent_;

    static constexpr const char* ADAP_OPT = "adaptation_options";
    static constexpr const char* ADAP_STRAT = "adaptation_strategy";
    static constexpr const char* ADAP_SUB = "adaptation_subject";
    static constexpr const char* ADAP_LOC = "subject_location";

    double evaluate_adaptation(aal_msgs::msg::Adaptation given_adaptation)
    {
      switch(given_adaptation.adaptation_target)
      {
        case static_cast<int8_t>(AdaptationTarget::RosParameter):
          return utility_of_adaptation(given_adaptation.parameter_adaptation);

        case static_cast<int8_t>(AdaptationTarget::LifecycleTransition):
          return utility_of_adaptation(given_adaptation.lifecycle_adaptation);
        
        default:
          std::cout << "no case could satisfy" << std::endl; 
      }

      return -1.0;
    }

    virtual double utility_of_adaptation(rcl_interfaces::msg::Parameter /*ros_parameter*/)
    {
      //Meant for external adaptation.
      return -1.0;
    }

    virtual double utility_of_adaptation(lifecycle_msgs::msg::Transition /*ros_lc_transition*/)
    {
      //Meant for external adaptation.
      return -1.0;
    }

    template <typename AdaptationService, typename AdaptationRequest>
    void sendAdaptationRequest(const std::string& service_name,
    const std::string& strategy_name,
    typename rclcpp::Client<AdaptationService>::SharedPtr& client,
    const std::string& registration_name)
    {
      client = node_->create_client<AdaptationService>(service_name, rmw_qos_profile_services_default, callback_group_);
      auto request = std::make_shared<AdaptationRequest>();

      if constexpr (std::is_same_v<AdaptationService, aal_msgs::srv::AdaptArchitectureExternal>)
      {
        request->adaptation_space = _var_params;
        request->task_identifier = registration_name;
        request->adaptation_strategy = strategy_name;
        request->utility_previous = _current_utilities;
        external_future_response_ = client->async_send_request(request).share();

        
      }
      else if constexpr (std::is_same_v<AdaptationService, aal_msgs::srv::AdaptArchitecture>)
      {
        request->adaptations = _internal_adaptations;
        std::cout << "internal adap sent type " << _internal_adaptations[0].parameter_adaptation.value.type << std::endl;
        internal_future_response_ = client->async_send_request(request).share();
      }
    }

    template <typename AdaptationService>
    bool receiveAdaptationRequest(const rclcpp::FutureReturnCode& ret)
    {

        auto const timeout = rclcpp::Duration::from_seconds( double(service_timeout_.count()) / 1000);

        if (ret != rclcpp::FutureReturnCode::SUCCESS)
        {
          if( (node_->now() - time_request_sent_) > timeout )
          {
            throw std::runtime_error("ran out of time trying to request adaptation, is your adaptation logic working properly? "); 
          }
          else{
            return false;
          }
        }
        else
        {
          response_received_ = true;

          if constexpr (std::is_same_v<AdaptationService, aal_msgs::srv::AdaptArchitectureExternal>){
            std::cout << "Response received!" << std::endl;

            external_response_ = external_future_response_.get();
            std::cout << "Got response from future!" << std::endl;

            // future_response_ = {};

            //You could check response success here

            if(!external_response_->success)
            {
              std::cout << "Warning: Unsuccessful adaptation" << std::endl;
            }
            else
            {
              std:: cout << "filling utilities" << std::endl;
              _current_utilities = {};
              for (auto const & adaptation : external_response_->applied_adaptations){
                _current_utilities.push_back(evaluate_adaptation(adaptation));
              }

              std:: cout << "filling utilities " << _current_utilities[0] << std::endl;

            
            }

            if (!external_response_) {
              throw std::runtime_error("Request was rejected by the service");
            }
          }
          else if constexpr (std::is_same_v<AdaptationService, aal_msgs::srv::AdaptArchitecture>){
            std::cout << "Response received!" << std::endl;

            internal_response_ = internal_future_response_.get();
            std::cout << "Got response from future!" << std::endl;

            // future_response_ = {};

            //You could check response success here

            if (!internal_response_) {
              throw std::runtime_error("Request was rejected by the service");
            }
          }

          return true;
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

        aal_msgs::msg::AdaptationOptions variable_param = aal_msgs::msg::AdaptationOptions();
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

        if(adaptation_type_ == AdaptationType::External)
        {
          sendAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal, aal_msgs::srv::AdaptArchitectureExternal::Request>(
            "/adapt_architecture_external", strategy_name, external_adapt_client_, this->registrationName());
            
        }
        else if(adaptation_type_ == AdaptationType::Internal)
        {
          sendAdaptationRequest<aal_msgs::srv::AdaptArchitecture, aal_msgs::srv::AdaptArchitecture::Request>(
            "/adapt_architecture", strategy_name, internal_adapt_client_, this->registrationName());
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
       
        rclcpp::FutureReturnCode ret;

        bool received = false;
        if(adaptation_type_ == AdaptationType::External){
          ret = callback_group_executor_.spin_until_future_complete(external_future_response_, nodelay);
          received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal>(ret);
        }
        else if(adaptation_type_ == AdaptationType::Internal){
          ret = callback_group_executor_.spin_until_future_complete(internal_future_response_, nodelay);
          received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitecture>(ret);
        }

        if(!received){
          return NodeStatus::RUNNING;
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
    
    

  private:







    bool adapted_yet = false;

    // std::shared_future<aal_msgs::srv::RequestAdaptation::Response::SharedPtr> adapt_result;
    // rclcpp::detail::FutureAndRequestId<std::future<SharedResponse>>
    // rclcpp::detail::FutureAndRequestId<std::future<aal_msgs::srv::RequestAdaptation::Response::SharedPtr>> adapt_result;
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
      
      // auto request = std::make_shared<aal_msgs::srv::RequestAdaptation::Request>();

      // future_response_ = {};
      // request->adaptation_options = _var_params;
      // request->task_identifier = this->registrationName();
      // request->adaptation_strategy = strategy_name;
      // request->adaptation_target = adaptation_target_;
      // request->lifecycle_transitions = _transitions;

      
      // future_response_ = adapt_client_->async_send_request(request).share();

        if(adaptation_type_ == AdaptationType::External)
        {
          sendAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal, aal_msgs::srv::AdaptArchitectureExternal::Request>(
            "/adapt_architecture_external", strategy_name, external_adapt_client_, this->registrationName());
            
        }
        else if(adaptation_type_ == AdaptationType::Internal)
        {
          sendAdaptationRequest<aal_msgs::srv::AdaptArchitecture, aal_msgs::srv::AdaptArchitecture::Request>(
            "/adapt_architecture", strategy_name, internal_adapt_client_, this->registrationName());
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

        rclcpp::FutureReturnCode ret;

        bool received = false;
        if(adaptation_type_ == AdaptationType::External){
          ret = callback_group_executor_.spin_until_future_complete(external_future_response_, nodelay);
          received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal>(ret);
        }
        else if(adaptation_type_ == AdaptationType::Internal){
          ret = callback_group_executor_.spin_until_future_complete(internal_future_response_, nodelay);
          received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitecture>(ret);
        }

        if(!received){
          return NodeStatus::RUNNING;
        }
        else
        {
          _to_adapt = false; //Reset condition.
          internal_response_ = {};
          external_response_ = {};
          request_sent_ = false;
        }
        
      }

    }

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

        if(adaptation_type_ == AdaptationType::External)
        {
          sendAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal, aal_msgs::srv::AdaptArchitectureExternal::Request>(
            "/adapt_architecture_external", strategy_name, external_adapt_client_, this->registrationName());
            
        }
        else if(adaptation_type_ == AdaptationType::Internal)
        {
          sendAdaptationRequest<aal_msgs::srv::AdaptArchitecture, aal_msgs::srv::AdaptArchitecture::Request>(
            "/adapt_architecture", strategy_name, internal_adapt_client_, this->registrationName());
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
           
            rclcpp::FutureReturnCode ret;

            bool received = false;
            if(adaptation_type_ == AdaptationType::External){
              ret = callback_group_executor_.spin_until_future_complete(external_future_response_, nodelay);
              received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal>(ret);
            }
            else if(adaptation_type_ == AdaptationType::Internal){
              ret = callback_group_executor_.spin_until_future_complete(internal_future_response_, nodelay);
              received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitecture>(ret);
            }

            if(!received){
              return NodeStatus::RUNNING;
            }
            else
            {
              _to_adapt = false; //Reset condition.

              internal_response_ = {};
              external_response_ = {};
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

          if(adaptation_type_ == AdaptationType::External)
          {
            sendAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal, aal_msgs::srv::AdaptArchitectureExternal::Request>(
              "/adapt_architecture_external", strategy_name, external_adapt_client_, this->registrationName());
              
          }
          else if(adaptation_type_ == AdaptationType::Internal)
          {
            sendAdaptationRequest<aal_msgs::srv::AdaptArchitecture, aal_msgs::srv::AdaptArchitecture::Request>(
              "/adapt_architecture", strategy_name, internal_adapt_client_, this->registrationName());
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

            bool received = false;
            if(adaptation_type_ == AdaptationType::External){
              ret = callback_group_executor_.spin_until_future_complete(external_future_response_, nodelay);
              received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitectureExternal>(ret);
            }
            else if(adaptation_type_ == AdaptationType::Internal){
              ret = callback_group_executor_.spin_until_future_complete(internal_future_response_, nodelay);
              received = receiveAdaptationRequest<aal_msgs::srv::AdaptArchitecture>(ret);
            }

            if(received){
              std::cout << "Response received!" << std::endl;

              _to_adapt = false; //Reset condition.

              internal_response_ = {};
              external_response_ = {};
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

}   // namespace BT
