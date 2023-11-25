#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rebet_msgs/action/behavior_tree.hpp"

#include "rebet_msgs/srv/set_blackboard.hpp"
#include "rebet_msgs/srv/set_attributes_in_blackboard.hpp"
#include "rebet_msgs/srv/set_parameter_in_blackboard.hpp"
#include "rebet_msgs/srv/get_blackboard.hpp"
#include "rebet_msgs/srv/get_qr.hpp"
#include "rebet_msgs/srv/get_variable_params.hpp"
#include "rebet_msgs/srv/set_weights.hpp"
#include "rebet_msgs/msg/qr.hpp"
#include "rebet_msgs/msg/system_attribute_value.hpp"



#include "rebet/system_attribute_value.hpp"

#include "rebet/slam_action_node.h"
#include "rebet/identify_action_node.h"
#include "rebet/identifyobject_action_node.h"
#include "rebet/gotopose_action_node.h"
#include "rebet/initial_pose.h"
#include "rebet/filter_obstacles.h"
#include "rebet/camera_feed_node.h"


#include "rebet/qr_node.h"
#include "rebet/adapt_node.h"
#include "rebet/spiral_search.h"
#include "rebet/arm_motors.h"
#include "rebet/set_mavros_mode.h"
#include "rebet/confirm_mavros_status.h"







#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>


using namespace BT;
using std::placeholders::_1;
using std::placeholders::_2;


class Arborist : public rclcpp::Node
{
public:
  using SystemAttributeValueMsg = rebet_msgs::msg::SystemAttributeValue;
  using SetAttributesInBlackboard = rebet_msgs::srv::SetAttributesInBlackboard;
  using SetParameterInBlackboard = rebet_msgs::srv::SetParameterInBlackboard;
  using SetBlackboard = rebet_msgs::srv::SetBlackboard;
  using GetBlackboard = rebet_msgs::srv::GetBlackboard;
  using GetQR = rebet_msgs::srv::GetQR;
  using GetVParams = rebet_msgs::srv::GetVariableParams;
  using SetWeights = rebet_msgs::srv::SetWeights;
  using QR_MSG = rebet_msgs::msg::QR;
  using BTAction = rebet_msgs::action::BehaviorTree;
  using GoalHandleBTAction = rclcpp_action::ServerGoalHandle<BTAction>;
  const std::string MSN_MAX_OBJ_PS_NAME = "max_objects_per_second";
  const std::string ENG_MAX_PIC_PS_NAME = "max_pictures_per_second";
  const std::string TSK_WINDOW_LEN_NAME = "task_qa_window";
  const std::string POW_WINDOW_LEN_NAME = "power_qa_window";

  const std::string BT_NAME_PARAM = "bt_filename";
  const std::string EXP_NAME_PARAM = "experiment_name";

  

  BT::Tree tree;
  BehaviorTreeFactory factory;
  std::string bt_name;
  std::string experiment_name;
  
  Arborist(std::string name = "arborist_node") : Node(name)
  { 
      _set_param_in_blackboard = this->create_service<SetParameterInBlackboard>("set_parameter_in_blackboard", std::bind(&Arborist::handle_set_param_bb, this, _1, _2));
      _set_att_in_blackboard = this->create_service<SetAttributesInBlackboard>("set_attributes_in_blackboard", std::bind(&Arborist::handle_set_atb_bb, this, _1, _2));
      _set_blackboard = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _get_blackboard = this->create_service<GetBlackboard>("get_blackboard", std::bind(&Arborist::handle_get_bb, this, _1, _2));
      _get_qr = this->create_service<GetQR>("get_qr", std::bind(&Arborist::handle_get_qr, this, _1, _2));
      _get_var_param = this->create_service<GetVParams>("get_variable_params", std::bind(&Arborist::handle_get_var_params, this, _1, _2));
      _set_weights = this->create_service<SetWeights>("set_weights", std::bind(&Arborist::handle_set_weights, this, _1, _2));

      // may throw ament_index_cpp::PackageNotFoundError exception
      std::string tree_dir = ament_index_cpp::get_package_share_directory("rebet") + "/trees/";
      std::cout << tree_dir << std::endl;

      //_start_tree = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _start_tree = rclcpp_action::create_server<BTAction>(
      this,
      "start_tree",
      std::bind(&Arborist::handle_tree_goal, this, _1, _2),
      std::bind(&Arborist::handle_tree_cancel, this, _1),
      std::bind(&Arborist::handle_tree_accepted, this, _1));

      //I suppose here you register all the possible custom nodes, and the determination as to whether they are actually used lies in the xml tree provided.

      registerActionClient<SLAMAction>(factory, "bt_slam_client", "slam", "SLAMfd");
      registerActionClient<IdentifyAction>(factory, "bt_identify_client", "identify", "IDfd");
      registerActionClient<IdentifyObjectAction>(factory, "bt_identifyobject_client", "checkForObjectsActionName", "identifyObject");
      
      registerActionClient<SpiralSearch>(factory, "bt_spiral_client", "spiral", "SpiralSearch");
      

      registerTopicClient<ConfirmMavrosStatus>(factory, "bt_mavros_status_sub", "ConfirmMavrosState");
      registerServiceClient<SetMavrosMode>(factory,"bt_mavrosmode_cli","SetMavrosMode");
      registerServiceClient<ArmMotors>(factory,"bt_armmotors_cli","ArmMotors");



      registerActionClient<VisitObstacleAction>(factory, "bt_gotopose_client", "navigate_to_pose", "visitObs");
      registerTopicClient<ProvideInitialPose>(factory,"bt_initialpose_pub","initialPose");
      registerTopicClient<CameraFeedNode>(factory,"bt_camera_sub","CameraFeed");
      registerServiceClient<FilterObstacles>(factory,"bt_filtersobs_cli","filterObstacles");
      
      factory.registerNodeType<TaskEfficiencyQR>("TaskEfficiencyQR");
      factory.registerNodeType<PowerQR>("PowerQR");
      factory.registerNodeType<SearchEfficiencyQR>("SearchEfficiencyQR");
      factory.registerNodeType<AdaptPictureRate>("AdaptPictureRate");
      factory.registerNodeType<AdaptSpiralAltitude>("AdaptSpiralAltitude");
      factory.registerNodeType<AdaptThrusterRecovery>("AdaptThrusterRecovery");




      this->declare_parameter(BT_NAME_PARAM, "spiraltest.xml");
      this->declare_parameter(EXP_NAME_PARAM, "no_experiment_name");

      bt_name = this->get_parameter(BT_NAME_PARAM).as_string();
      experiment_name = this->get_parameter(EXP_NAME_PARAM).as_string();

      RCLCPP_INFO(this->get_logger(), bt_name.c_str());

      tree = factory.createTreeFromFile(tree_dir + bt_name);

      this->declare_parameter(MSN_MAX_OBJ_PS_NAME, 0.14);
      this->declare_parameter(ENG_MAX_PIC_PS_NAME, 0.4);
      this->declare_parameter(TSK_WINDOW_LEN_NAME, 8);
      this->declare_parameter(POW_WINDOW_LEN_NAME, 8);
      
      


      double max_objs_ps = this->get_parameter(MSN_MAX_OBJ_PS_NAME).as_double();

      float max_pics_ps = this->get_parameter(ENG_MAX_PIC_PS_NAME).as_double();
      int msn_window_length = this->get_parameter(TSK_WINDOW_LEN_NAME).as_int();
      int eng_window_length = this->get_parameter(POW_WINDOW_LEN_NAME).as_int();



      auto node_visitor = [max_objs_ps, msn_window_length, eng_window_length, max_pics_ps](TreeNode* node)
      {
        if (auto task_qr_node = dynamic_cast<TaskEfficiencyQR*>(node))
        {
          task_qr_node->initialize(max_objs_ps,
                                       msn_window_length);
        }
        if (auto power_qr_node = dynamic_cast<PowerQR*>(node))
        {
          power_qr_node->initialize(max_pics_ps,
                                       eng_window_length);
        }
        if (auto search_qr_node = dynamic_cast<SearchEfficiencyQR*>(node))
        {
          search_qr_node->initialize(5);
        }
      };

      

      // Apply the visitor to ALL the nodes of the tree
      tree.applyVisitor(node_visitor);

  }

  template <class T>
  void registerActionClient(BehaviorTreeFactory& factory, std::string client_name, std::string action_name, std::string name_in_xml)
  {
    auto options = rclcpp::NodeOptions().use_global_arguments(false); //https://answers.ros.org/question/316870/ros2-composition-and-node-names-with-launch-files/?answer=316925#post-id-316925
    auto nh = std::make_shared<rclcpp::Node>(client_name, options);



    RosNodeParams params;
    params.nh = nh;
    params.default_port_value = action_name;
    params.server_timeout = std::chrono::milliseconds(4000); //This resolves a race condition with the ServerGoalTimeout (Error 1 in RosActionNode) I suppose caused by the size of the system.
    factory.registerNodeType<T>(name_in_xml, params);

  }

  template <class T>
  void registerTopicClient(BehaviorTreeFactory& factory, std::string publisher_name, std::string name_in_xml)
  {
    auto options = rclcpp::NodeOptions().use_global_arguments(false); //https://answers.ros.org/question/316870/ros2-composition-and-node-names-with-launch-files/?answer=316925#post-id-316925
    auto nh = std::make_shared<rclcpp::Node>(publisher_name, options);

    RosNodeParams params;
    params.nh = nh;
    factory.registerNodeType<T>(name_in_xml, params);
  }

  template <class T>
  void registerServiceClient(BehaviorTreeFactory& factory, std::string service_name, std::string name_in_xml)
  {
    auto options = rclcpp::NodeOptions().use_global_arguments(false); //https://answers.ros.org/question/316870/ros2-composition-and-node-names-with-launch-files/?answer=316925#post-id-316925
    auto nh = std::make_shared<rclcpp::Node>(service_name, options);

    RosNodeParams params;
    params.nh = nh;
    factory.registerNodeType<T>(name_in_xml, params);

  }
  

private:

  bool inject_script_node(std::string script)
  {
    std::string _script;
    ScriptFunction _executor;

    auto executor = ParseScript(script);
    if (!executor)
    {
      return false;

      throw RuntimeError(executor.error());
      
    }
    else
    {
      _executor = executor.value();
      _script = script;
    }

    if (_executor)
    {
      Ast::Environment env = {tree.rootBlackboard(), nullptr};
      _executor(env);
    }

    return true;
    
  }



  void handle_set_atb_bb(const std::shared_ptr<SetAttributesInBlackboard::Request> request,
        std::shared_ptr<SetAttributesInBlackboard::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Set Attributes Service Called in Arborist Node");

    for (auto const & sys_attr : request->sys_attributes)
    {
      auto sys_attvalue_obj = rebet::SystemAttributeValue(sys_attr.value);
      tree.rootBlackboard()->set<rebet::SystemAttributeValue>(sys_attr.name, sys_attvalue_obj);
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
        tree.rootBlackboard()->set<std::vector<int>>(name, convertedValue);
    }
    else
    {
      tree.rootBlackboard()->set<T>(name, value.get<T>());
    }
  }

  void handle_set_param_bb(const std::shared_ptr<SetParameterInBlackboard::Request> request,
        std::shared_ptr<SetParameterInBlackboard::Response> response)
  {
    RCLCPP_INFO(this->get_logger(), "Set Parameter Service Called in Arborist Node");

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






  void handle_set_bb(const std::shared_ptr<SetBlackboard::Request> request,
        std::shared_ptr<SetBlackboard::Response> response)
  {
    std::cout<<"Set BlackBoard Service Called in Arborist Node"<<std::endl;

    std::string script = request->script_code;
    
    response->success = inject_script_node(script);
  }

  void handle_get_bb(const std::shared_ptr<GetBlackboard::Request> request,
        std::shared_ptr<GetBlackboard::Response> response)
  {
    std::cout << "Service to get a value from the blackboard" << std::endl;

    const std::string sought_key = request->key_name;

    std::cout << "sought key" << sought_key << std::endl;
    //this get isn't safe for the value not being assigned yet..
    auto entry_value = tree.rootBlackboard()->get<std::string>(sought_key);

    std::cout << "the value is " << entry_value << " gotten from the bb " << std::endl;

    response->key_value = entry_value;
  }

  std::vector<QRNode*> get_tree_qrs()
  {    
    std::cout << "2 Service to get a qr nodes from the blackboard" << std::endl;

    std::vector<QRNode*> qr_nodes;

    for (auto const & sbtree : tree.subtrees) 
    {
      for (auto & node : sbtree->nodes) 
      {
        std::cout << "Node is ?" << node->name() << node->status() << std::endl;

        if(auto qr_node = dynamic_cast<QRNode*>(static_cast<TreeNode*>(node.get())))
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

  std::vector<TreeNode::Ptr> get_tree_variable_acs()
  {    
    std::cout << "2 Service to get a variable nodes from the blackboard" << std::endl;

    std::vector<TreeNode::Ptr> variable_decorator_nodes;

    for (auto const & sbtree : tree.subtrees) 
    {
      for (auto & node : sbtree->nodes) 
      {
        auto node_config = node->config();

        if(node->type() == NodeType::DECORATOR && (node_config.output_ports.find(AdaptNode::VARIABLE_PARAMS) != node_config.output_ports.end()))
        {
          variable_decorator_nodes.push_back(node);
        }
        // if(auto var_ac_node = dynamic_cast<ActionNodeBase*>(static_cast<TreeNode*>(node.get())))
        // {
        //   variable_action_nodes.push_back(var_ac_node);
        // }
      }
    }
    std::cout << "2 End Service to get a variable nodes from the blackboard " << variable_decorator_nodes.size() << std::endl;

    return variable_decorator_nodes;
  }
  
  void handle_get_qr(const std::shared_ptr<GetQR::Request> request,
        std::shared_ptr<GetQR::Response> response)
  {
    std::cout << "Service to get a QR nodes from the blackboard" << std::endl;

    auto qr_nodes = get_tree_qrs();
    for (auto & node : qr_nodes) 
    {
      if(node->status() == NodeStatus::RUNNING) //ensures that the QRs are currently in effect.
      {
      auto node_config = node->config();

      auto weight = node_config.input_ports.find(QRNode::WEIGHT);
      auto metric = node_config.output_ports.find(QRNode::METRIC);

      if (weight == node_config.input_ports.end() || metric == node_config.output_ports.end())
      {
        std::stringstream ss;
        ss << "Either port weight port " << QRNode::WEIGHT << " or metric port " << QRNode::METRIC << " not found within the QR node " << node->registrationName();
        RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
        return;
      }

      auto metric_bb_value = tree.rootBlackboard()->get<double>((std::string)TreeNode::stripBlackboardPointer(metric->second));
      auto weight_bb_value = tree.rootBlackboard()->get<double>((std::string)TreeNode::stripBlackboardPointer(weight->second));

      QR_MSG qr_msg;
      qr_msg.qr_name = node->registrationName();
      qr_msg.metric = metric_bb_value; 
      qr_msg.weight = weight_bb_value;

      response->qrs_in_tree.push_back(qr_msg);
      }
    }
    std::cout << "End service to get a QR nodes from the blackboard " << response->qrs_in_tree.size() << std::endl;

  }

  void handle_get_var_params(const std::shared_ptr<GetVParams::Request> request,
        std::shared_ptr<GetVParams::Response> response)
  {
    std::cout << "1 Service to get a variable nodes from the blackboard" << std::endl;
  
    auto var_nodes = get_tree_variable_acs();
    for (auto & node : var_nodes) 
    {
      if(node->status() == NodeStatus::RUNNING) //ensures that the actions are currently in effect.
      {
      auto node_config = node->config();

      auto var_params = node_config.output_ports.find(VariableActionNodeBase::VARIABLE_PARAMS); //this is safe because the previous method guarantees the presence of this port.

      // if (var_params == node_config.output_ports.end())
      // {
      //   std::stringstream ss;
      //   ss << "Output port " << VariableActionNode<ActionT>::VARIABLE_PARAMS << " not found within the variable action node " << node->registrationName();
      //   RCLCPP_ERROR(this->get_logger(), ss.str().c_str());
      //   return;
      // }

      auto var_bb_value = tree.rootBlackboard()->get<VariableParameters>((std::string)TreeNode::stripBlackboardPointer(var_params->second));
      //Each variable action node can provide one or more changeable parameters.

      for (auto var_param : var_bb_value.variable_parameters)
      {
        response->variables_in_tree.variable_parameters.push_back(var_param); //We put all of these parameters that may change into a singular list. 
        //Potentially we may want a list of these lists to keep them distinct.. 
      }
      }
    }
      std::cout << "1 End Service to get a variable nodes from the blackboard " << response->variables_in_tree.variable_parameters.size() << std::endl;

  }
  

  

  void handle_set_weights(const std::shared_ptr<SetWeights::Request> request,
        std::shared_ptr<SetWeights::Response> response)
  {
    std::string script = "";
    auto qr_nodes = get_tree_qrs();
    auto qr_msgs = request->qrs_to_update;


    for (QR_MSG & qr_msg : qr_msgs)
    {
      std::string qr_name = qr_msg.qr_name;
    
      for (auto & node : qr_nodes) 
      {
        if(qr_name == node->registrationName())
        {
          auto node_config = node->config();

          auto weight = node_config.input_ports.find(QRNode::WEIGHT);

          if (weight == node_config.input_ports.end())
          {
            std::cout << "weight not found" << std::endl;
            return;
          }
          
          std::string weight_bb_key = (std::string)TreeNode::stripBlackboardPointer(weight->second);

          float weight_as_float = (float)qr_msg.weight;
          script += weight_bb_key;
          script += ":=";
          script += std::to_string(qr_msg.weight);
          script += "; ";
        }

      }

    }

    script.pop_back();
    script.pop_back(); //Removing the last '; ' as it isn't necessary.

    response->success = inject_script_node(script);

  }



  void handle_start_tree(const std::shared_ptr<GoalHandleBTAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal: Starting to tick tree");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<BTAction::Feedback>();
    feedback->node_status = "placeholder";

    auto result = std::make_shared<BTAction::Result>();

    NodeStatus result_of_tick;


    do {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->is_success = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled: didn't finish BT");
        return;
      }

      try{
        result_of_tick = tree.tickOnce();
      }
      catch (const std::runtime_error& error)
      {
        //If an error is thrown then we just consider the tree to have failed.
        
        RCLCPP_INFO(this->get_logger(), "Tree failed because of runtime error.");
        RCLCPP_ERROR(this->get_logger(), error.what());

        result_of_tick = NodeStatus::FAILURE;
      }

      feedback->node_status = toStr(result_of_tick);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      // RCLCPP_INFO(this->get_logger(), "Publish feedback");

      // Check if goal is done
      if(result_of_tick != NodeStatus::RUNNING) {
        if (rclcpp::ok()) {

          switch(result_of_tick) {
            case NodeStatus::SUCCESS:
              result->is_success = true;
              break;
            case NodeStatus::FAILURE:
              result->is_success = false;
              break;
          }
          goal_handle->succeed(result);
          RCLCPP_INFO(this->get_logger(), "Goal finished: Done ticking the tree");
          
          // //Reporting on mission
          // float id_time_elapsed = tree.rootBlackboard()->get<float>("id_time_elapsed");
          // int32_t id_picture_rate = tree.rootBlackboard()->get<int32_t>("id_picture_rate");
          // float avg_task_metric = tree.rootBlackboard()->get<float>("task_mean_metric");
          // float avg_power_metric = tree.rootBlackboard()->get<float>("power_mean_metric");

          // int32_t id_det_threshold = tree.rootBlackboard()->get<int32_t>("id_det_threshold");
          // auto average_utility = tree.rootBlackboard()->get<std::string>("average_utility");


          
          // auto curr_time_pointer = std::chrono::system_clock::now();
          // int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();

          // // file pointer
          // std::fstream fout;
        
          // // opens an existing csv file or creates a new file.
          // fout.open("rebet_results.csv", std::ios::out | std::ios::app);
        
          // //Header
          // // fout << "timestamp" << ", " 
          // //     << "picture_rate" << ", "
          // //     << "time_elapsed" << "\n";

          // // Insert the data to file
          // fout << current_time << ", " 
          //     << id_picture_rate << ", "
          //     << avg_task_metric << ", "
          //     << avg_power_metric << ", "
          //     << id_time_elapsed << ", "
          //     << id_det_threshold << ", "
          //     << experiment_name << ", "
          //     << bt_name << ", "
          //     << average_utility << "\n";

            
          // fout.close();



          // std::ofstream outfile ("mission.done");

          // outfile << "." << std::endl;

          // outfile.close();

          break;
        }

      }
    }
    while(result_of_tick == NodeStatus::RUNNING);
  }


  rclcpp_action::GoalResponse handle_tree_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const BTAction::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_tree_cancel(
    const std::shared_ptr<GoalHandleBTAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_tree_accepted(const std::shared_ptr<GoalHandleBTAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Handle accepted callback");

    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&Arborist::handle_start_tree, this, _1), goal_handle}.detach();
  }

  rclcpp::Service<SetBlackboard>::SharedPtr _set_blackboard;
  rclcpp::Service<GetBlackboard>::SharedPtr _get_blackboard;
  rclcpp::Service<GetQR>::SharedPtr _get_qr;
  rclcpp::Service<GetVParams>::SharedPtr _get_var_param;
  rclcpp::Service<SetWeights>::SharedPtr _set_weights;
  rclcpp::Service<SetAttributesInBlackboard>::SharedPtr _set_att_in_blackboard;
  rclcpp::Service<SetParameterInBlackboard>::SharedPtr _set_param_in_blackboard;



  rclcpp_action::Server<BTAction>::SharedPtr _start_tree; 
};




int main(int argc, char * argv[])
{
  std::cout << "Now I'm here and I wish that I wasn't";
  

  rclcpp::init(argc, argv);

  auto node = std::make_shared<Arborist>();
  rclcpp::spin(node);
  rclcpp::shutdown();




  //return 0;
}
