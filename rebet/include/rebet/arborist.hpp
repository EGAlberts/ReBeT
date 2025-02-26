#ifndef rebet__ARBORIST_HPP_
#define rebet__ARBORIST_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "rebet_msgs/srv/set_blackboard.hpp"
#include "rebet_msgs/srv/set_attributes_in_blackboard.hpp"
#include "rebet_msgs/srv/set_parameter_in_blackboard.hpp"
#include "rebet_msgs/srv/get_blackboard.hpp"
#include "rebet_msgs/srv/get_qr.hpp"
#include "rebet_msgs/srv/set_weights.hpp"
#include "rebet_msgs/msg/qr.hpp"
#include "rebet_msgs/msg/system_attribute_value.hpp"
#include "rebet/rebet_utilities.hpp"


#include "rebet/system_attribute_value.hpp"
#include "rebet/qr_node.h"
#include "rebet/adapt_node.h"

#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_ros2/tree_execution_server.hpp>

using namespace BT;
using std::placeholders::_1;
using std::placeholders::_2;


class Arborist : public BT::TreeExecutionServer
{
public:
  using SystemAttributeValueMsg = rebet_msgs::msg::SystemAttributeValue;
  using SetAttributesInBlackboard = rebet_msgs::srv::SetAttributesInBlackboard;
  using SetParameterInBlackboard = rebet_msgs::srv::SetParameterInBlackboard;
  using GetBlackboard = rebet_msgs::srv::GetBlackboard;
  using GetQR = rebet_msgs::srv::GetQR;
  using SetWeights = rebet_msgs::srv::SetWeights;
  using QR_MSG = rebet_msgs::msg::QR;
  const std::string MSN_MAX_OBJ_PS_NAME = "max_objects_per_second";
  const std::string ENG_MAX_PIC_PS_NAME = "max_pictures_per_second";
  const std::string TSK_WINDOW_LEN_NAME = "task_qa_window";
  const std::string POW_WINDOW_LEN_NAME = "power_qa_window";

  const std::string BT_NAME_PARAM = "bt_filename";

  

  BT::Tree tree;
  BehaviorTreeFactory factory;
  std::string bt_name;

  
  Arborist(const rclcpp::NodeOptions& options) : TreeExecutionServer(options)
  { 
      _set_param_in_blackboard = node()->create_service<SetParameterInBlackboard>("set_parameter_in_blackboard", std::bind(&Arborist::handle_set_param_bb, this, _1, _2));
      _set_att_in_blackboard = node()->create_service<SetAttributesInBlackboard>("set_attributes_in_blackboard", std::bind(&Arborist::handle_set_atb_bb, this, _1, _2));
      _get_blackboard = node()->create_service<GetBlackboard>("get_blackboard", std::bind(&Arborist::handle_get_bb, this, _1, _2));
      _get_qr = node()->create_service<GetQR>("get_qr", std::bind(&Arborist::handle_get_qr, this, _1, _2));
      _set_weights = node()->create_service<SetWeights>("set_weights", std::bind(&Arborist::handle_set_weights, this, _1, _2));
      
      std::cout << "parent arborist construction complete" << std::endl;
  }

  bool onGoalReceived(const std::string& tree_name, const std::string& payload) override
  {
    bt_name = tree_name;
    return true;
  }

protected:

  // bool inject_script_node(std::string script)
  // {
  //   std::string _script;
  //   ScriptFunction _executor;

  //   auto executor = ParseScript(script);
  //   if (!executor)
  //   {
  //     return false;

  //     throw RuntimeError(executor.error());
      
  //   }
  //   else
  //   {
  //     _executor = executor.value();
  //     _script = script;
  //   }

  //   if (_executor)
  //   {
  //     Ast::Environment env = {tree.rootBlackboard(), nullptr};
  //     _executor(env);
  //   }

  //   return true;
    
  // }

  void handle_set_atb_bb(const std::shared_ptr<SetAttributesInBlackboard::Request> request,
        std::shared_ptr<SetAttributesInBlackboard::Response> response)
  { 
    for (auto const & sys_attr : request->sys_attributes)
    {
      auto sys_attvalue_obj = rebet::SystemAttributeValue(sys_attr.value);
      globalBlackboard()->set(sys_attr.name, sys_attvalue_obj);
    }
    response->success = true;
  }

  template <class T>
  void set_param_in_bb(const std::string name, const rclcpp::ParameterValue value)
  {
    if constexpr (std::is_same_v<T, std::vector<int>>) {
        // Convert from int64_t to int if necessary
        const std::vector<int64_t>& originalValue = value.get<std::vector<int64_t>>();
        std::vector<int> convertedValue(originalValue.begin(), originalValue.end());
        // tree.rootBlackboard()->set<std::vector<int>>(name, convertedValue);
        globalBlackboard()->set(name, convertedValue);

    }
    else
    {
      // tree.rootBlackboard()->set<T>(name, value.get<T>());
      globalBlackboard()->set(name, value.get<T>());
    }
  }

 void handle_set_param_bb(const std::shared_ptr<SetParameterInBlackboard::Request> request,
        std::shared_ptr<SetParameterInBlackboard::Response> response)
  {
    // RCLCPP_INFO(node()->get_logger(), "Set Parameter Service Called in Arborist Node");

    for (auto const & ros_parameter : request->ros_parameters)
    {
      auto ros_param_value_obj = rclcpp::ParameterValue(ros_parameter.value);

      switch(ros_param_value_obj.get_type())
      {
        case rclcpp::ParameterType::PARAMETER_NOT_SET:
          response->success = false;
          throw std::runtime_error("Parameter meant for blackboard did not have value set");
          break;

        case rclcpp::ParameterType::PARAMETER_BOOL:
          set_param_in_bb<bool>(ros_parameter.name, ros_param_value_obj);
          break;
        case rclcpp::ParameterType::PARAMETER_INTEGER:
          set_param_in_bb<int>(ros_parameter.name, ros_param_value_obj);
          break;
        case rclcpp::ParameterType::PARAMETER_DOUBLE:
          set_param_in_bb<double>(ros_parameter.name, ros_param_value_obj);
          break;      
        case rclcpp::ParameterType::PARAMETER_STRING:
          set_param_in_bb<std::string>(ros_parameter.name, ros_param_value_obj);
          break;      
        case rclcpp::ParameterType::PARAMETER_BYTE_ARRAY:
          set_param_in_bb<std::vector<uint8_t>>(ros_parameter.name, ros_param_value_obj);
          break;      
        case rclcpp::ParameterType::PARAMETER_BOOL_ARRAY:
          set_param_in_bb<std::vector<bool>>(ros_parameter.name, ros_param_value_obj);
          break;      
        case rclcpp::ParameterType::PARAMETER_INTEGER_ARRAY:
          set_param_in_bb<std::vector<int>>(ros_parameter.name, ros_param_value_obj);
          break;      
        case rclcpp::ParameterType::PARAMETER_DOUBLE_ARRAY:
          set_param_in_bb<std::vector<double>>(ros_parameter.name, ros_param_value_obj);
          break;      
        case rclcpp::ParameterType::PARAMETER_STRING_ARRAY:
          set_param_in_bb<std::vector<std::string>>(ros_parameter.name, ros_param_value_obj);
          break;
      }
    }
    response->success = true;
  }


  void handle_get_bb(const std::shared_ptr<GetBlackboard::Request> request,
        std::shared_ptr<GetBlackboard::Response> response)
  {
    std::cout << "Service to get a value from the blackboard" << std::endl;

    const std::string sought_key = request->key_name;

    std::cout << "sought key" << sought_key << std::endl;
    //this get isn't safe for the value not being assigned yet..
    auto entry_value = globalBlackboard()->get<std::string>(sought_key);

    std::cout << "the value is " << entry_value << " gotten from the bb " << std::endl;

    response->key_value = entry_value;
  }

  template <class QR_TYPE>
  std::vector<QR_TYPE*> get_tree_qrs()
  {    
    std::vector<QR_TYPE*> qr_nodes = {};

    for (auto const & sbtree : tree.subtrees) 
    {
      for (auto & node : sbtree->nodes) 
      {
        std::cout << "Node is ?" << node->name() << node->status() << std::endl;

        if(auto qr_node = dynamic_cast<QR_TYPE*>(static_cast<TreeNode*>(node.get())))
        {
          std::cout << "QR node" << std::endl;

          qr_nodes.push_back(qr_node);
        }
        else{
          std::cout << "Not QR" << std::endl;


        }
      }
    }
    return qr_nodes;
    std::cout << "2 End Service to get a qr nodes from the blackboard " << qr_nodes.size() << std::endl;

  }

  template <class QR_TYPE>  
  std::vector<QR_MSG> create_qr_msgs(std::vector<QR_TYPE*> qr_nodes)
  {
    std::vector<QR_MSG> qr_msgs = {};
    for (auto & node : qr_nodes) 
    {
      if(node->status() == NodeStatus::RUNNING) //ensures that the QRs are currently in effect.
      {

      // auto metric_bb_value = globalBlackboard()->get<double>((std::string)TreeNode::stripBlackboardPointer(metric->second));
      // auto weight_bb_value = globalBlackboard()->get<double>((std::string)TreeNode::stripBlackboardPointer(weight->second));

      QR_MSG qr_msg;
      qr_msg.qr_name = node->registrationName();
      qr_msg.metric = node->current_metric();; 
      qr_msg.weight = 1.0; //weight_bb_value; We don't use the weights right now, add a public getter (or equivalent) somehow once needed.
      qr_msg.higher_is_better = node->is_higher_better();

      qr_msgs.push_back(qr_msg);
      }
    }
    return qr_msgs;
  }

  template <class QR_TYPE>
  bool set_weights_of_qrs(std::vector<QR_MSG> qr_msgs, std::vector<QR_TYPE*> qr_nodes)
  {    
    std::string script = "";
    // for (QR_MSG & qr_msg : qr_msgs)
    //   {
    //     std::string qr_name = qr_msg.qr_name;
      
    //     for (auto & node : qr_nodes) 
    //     {
    //       if(qr_name == node->registrationName())
    //       {
    //         auto node_config = node->config();

    //         auto weight = node_config.input_ports.find(QRNode::WEIGHT);

    //         if (weight == node_config.input_ports.end())
    //         {
    //           std::cout << "weight not found" << std::endl;
    //           return false;
    //         }
            
    //         std::string weight_bb_key = (std::string)TreeNode::stripBlackboardPointer(weight->second);

    //         script += weight_bb_key;
    //         script += ":=";
    //         script += std::to_string(qr_msg.weight);
    //         script += "; ";
    //       }

    //     }

    //   } TODO: Restore, config() is now protected.

      script.pop_back();
      script.pop_back(); //Removing the last '; ' as it isn't necessary.

      return false; //TODO: RESTORE THIS FUNCTIONALITY
      //return inject_script_node(script);

  }


  
  void handle_get_qr(const std::shared_ptr<GetQR::Request> request,
        std::shared_ptr<GetQR::Response> response)
  {
    std::cout << "Service to get a QR nodes from the blackboard" << std::endl;

    std::vector<SystemLevelQR*> sys_qr_nodes = {};
    std::vector<TaskLevelQR*> tsk_qr_nodes = {};

    if(request->at_system_level)
    {
      sys_qr_nodes = get_tree_qrs<SystemLevelQR>();
      response->qrs_in_tree = create_qr_msgs<SystemLevelQR>(sys_qr_nodes);
    }
    else
    {
      tsk_qr_nodes = get_tree_qrs<TaskLevelQR>();
      response->qrs_in_tree = create_qr_msgs<TaskLevelQR>(tsk_qr_nodes);

    }

    std::cout << "End service to get a QR nodes from the blackboard " << response->qrs_in_tree.size() << std::endl;

  }


  void handle_set_weights(const std::shared_ptr<SetWeights::Request> request,
        std::shared_ptr<SetWeights::Response> response)
  {
    std::vector<SystemLevelQR*> sys_qr_nodes;
    std::vector<TaskLevelQR*> task_qr_nodes;

    sys_qr_nodes = get_tree_qrs<SystemLevelQR>();
    
    task_qr_nodes = get_tree_qrs<TaskLevelQR>();

    std::vector<rebet_msgs::msg::QR> qr_msgs = request->qrs_to_update;

    response->success = (set_weights_of_qrs<SystemLevelQR>(qr_msgs,sys_qr_nodes) && set_weights_of_qrs<TaskLevelQR>(qr_msgs,task_qr_nodes));
 
  }

  rclcpp::Service<GetBlackboard>::SharedPtr _get_blackboard;
  rclcpp::Service<GetQR>::SharedPtr _get_qr;
  rclcpp::Service<SetWeights>::SharedPtr _set_weights;
  rclcpp::Service<SetAttributesInBlackboard>::SharedPtr _set_att_in_blackboard;
  rclcpp::Service<SetParameterInBlackboard>::SharedPtr _set_param_in_blackboard;
};
#endif  // rebet__ARBORIST_HPP_