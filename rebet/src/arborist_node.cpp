#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "rebet_msgs/action/behavior_tree.hpp"

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

#include "rebet_frog/gotopose_action_node.h"
#include "rebet_frog/visitobstacle_action_node.h"
#include "rebet_frog/spin_node.h"
#include "rebet_frog/initial_pose.h"
#include "rebet_frog/filter_obstacles.h"
#include "rebet_frog/get_map.h"
#include "rebet_frog/find_frontier.h"
#include "rebet_frog/dock_location.h"
#include "rebet_frog/identify_object.h"




#include "rebet_frog/frog_qrs.h"
#include "rebet_frog/frog_adapt_nodes.h"

#include "rebet/camera_feed_node.h"
#include "rebet/robot_pose_node.h"
#include "rebet/sleep_node.h"



#include "rebet/qr_node.h"
#include "rebet/adapt_node.h"

#include "behaviortree_cpp/loggers/groot2_publisher.h"



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
  using SetWeights = rebet_msgs::srv::SetWeights;
  using QR_MSG = rebet_msgs::msg::QR;
  using BTAction = rebet_msgs::action::BehaviorTree;
  using GoalHandleBTAction = rclcpp_action::ServerGoalHandle<BTAction>;
  const std::string MSN_MAX_OBJ_PS_NAME = "max_objects_per_second";
  const std::string ENG_MAX_PIC_PS_NAME = "max_pictures_per_second";
  const std::string TSK_WINDOW_LEN_NAME = "task_qa_window";
  const std::string POW_WINDOW_LEN_NAME = "power_qa_window";

  const std::string BT_NAME_PARAM = "bt_filename";

  

  BT::Tree tree;
  BehaviorTreeFactory factory;
  std::string bt_name;

  
  Arborist(std::string name = "arborist_node") : Node(name)
  { 
      _set_param_in_blackboard = this->create_service<SetParameterInBlackboard>("set_parameter_in_blackboard", std::bind(&Arborist::handle_set_param_bb, this, _1, _2));
      _set_att_in_blackboard = this->create_service<SetAttributesInBlackboard>("set_attributes_in_blackboard", std::bind(&Arborist::handle_set_atb_bb, this, _1, _2));
      _set_blackboard = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _get_blackboard = this->create_service<GetBlackboard>("get_blackboard", std::bind(&Arborist::handle_get_bb, this, _1, _2));
      _get_qr = this->create_service<GetQR>("get_qr", std::bind(&Arborist::handle_get_qr, this, _1, _2));
      _set_weights = this->create_service<SetWeights>("set_weights", std::bind(&Arborist::handle_set_weights, this, _1, _2));

      // may throw ament_index_cpp::PackageNotFoundError exception
      std::string tree_dir = ament_index_cpp::get_package_share_directory("rebet") + "/trees/";

      //_start_tree = this->create_service<SetBlackboard>("set_blackboard", std::bind(&Arborist::handle_set_bb, this, _1, _2));
      _start_tree = rclcpp_action::create_server<BTAction>(
      this,
      "start_tree",
      std::bind(&Arborist::handle_tree_goal, this, _1, _2),
      std::bind(&Arborist::handle_tree_cancel, this, _1),
      std::bind(&Arborist::handle_tree_accepted, this, _1));

      publisher_ = this->create_publisher<std_msgs::msg::String>("arborist/reporting", 10);

      //I suppose here you register all the possible custom nodes, and the determination as to whether they are actually used lies in the xml tree provided.

      // registerActionClient<IdentifyObjectAction>(factory, "bt_identifyobject_client", "checkForObjectsActionName", "identifyObject");
      
      registerActionClient<VisitObstacleAction>(factory, "bt_gotopose_client", "navigate_to_pose", "visitObs");
      
      registerTopicClient<ProvideInitialPose>(factory,"bt_initialpose_pub","initialPose");
      registerTopicClient<SlowlySpin>(factory,"bt_spin_pub","slowlySpin");
      registerTopicClient<CameraFeedNode>(factory,"bt_camera_sub","CameraFeed");
      registerTopicClient<RobotPoseNode>(factory,"bt_getpose_sub","getRobotPose");
      registerServiceClient<GetMap>(factory,"bt_getmap_cli","getMap");
      registerServiceClient<FindFrontier>(factory,"bt_findfrontier_cli","findFrontier");
      registerServiceClient<IdentifyObjectService>(factory,"bt_detobj_srv_cli","NewIDObj");

      registerActionClient<GoToPoseAction>(factory, "bt_gotopose_frontier_client", "navigate_to_pose", "visitFrontier");
      registerActionClient<GoToPoseAction>(factory, "bt_gotopose_dock_client", "navigate_to_pose", "visitChargingDock");
      
      factory.registerNodeType<FilterObstacles>("filterObstacles");
      factory.registerNodeType<SleepNode>("Sleep");

      
      factory.registerNodeType<ObjectDetectionEfficiencyQR>("DetectObjectsEfficiently");
      factory.registerNodeType<SimpleSystemPowerQR>("KeepBatteryMin");
      factory.registerNodeType<SystemPowerQR>("PowerQR");
      factory.registerNodeType<SafetyQR>("SafetyQR");
      factory.registerNodeType<ObjectDetectionPowerQR>("DetectObjectsSavePower");


      factory.registerNodeType<MovementEfficiencyQR>("MovementEfficiencyQR");
      factory.registerNodeType<MovementPowerQR>("MovementPowerQR");


      factory.registerNodeType<AdaptPictureRateExternal>("AdaptPictureRate");
      factory.registerNodeType<AdaptPictureRateInternal>("AdaptPictureRateOff");

      factory.registerNodeType<AdaptMaxSpeedExternal>("AdaptMaxSpeed");
      factory.registerNodeType<AdaptMaxSpeedInternal>("AdaptMaxSpeedOff");

      factory.registerNodeType<FromExploreToIdentify>("FromExploreToIdentify");


      // factory.registerNodeType<AdaptChargeConditionInternal>("WhetherToCharge");
      factory.registerNodeType<SetDockLocation>("SetChargingDockLocation");

      
      std::cout << "all nodes registered" << std::endl;


      this->declare_parameter(BT_NAME_PARAM, "frog_aal_arch_external.xml");

      bt_name = this->get_parameter(BT_NAME_PARAM).as_string();

      RCLCPP_INFO(this->get_logger(), bt_name.c_str());

      tree = factory.createTreeFromFile(tree_dir + bt_name);

      BT::Groot2Publisher publisher(tree);

      this->declare_parameter(MSN_MAX_OBJ_PS_NAME, 0.14);
      this->declare_parameter(ENG_MAX_PIC_PS_NAME, 0.4);
      this->declare_parameter(TSK_WINDOW_LEN_NAME, 8);
      this->declare_parameter(POW_WINDOW_LEN_NAME, 8);
      
      


      double max_objs_ps = this->get_parameter(MSN_MAX_OBJ_PS_NAME).as_double();

      float max_pics_ps = this->get_parameter(ENG_MAX_PIC_PS_NAME).as_double();
      int pow_window_length = this->get_parameter(POW_WINDOW_LEN_NAME).as_int();



      auto node_visitor = [max_objs_ps, pow_window_length, max_pics_ps](TreeNode* node)
      {
        if (auto power_qr_node = dynamic_cast<SystemPowerQR*>(node))
        {
          power_qr_node->initialize(max_pics_ps,
                                       pow_window_length);
        }
      };

      


      // Apply the visitor to ALL the nodes of the tree
      tree.applyVisitor(node_visitor);

      std::cout << "construction complete" << std::endl;

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
    params.server_timeout = std::chrono::milliseconds(YOLO_SERVICE_TIMEOUT_MILLISECOND); //The YOLO can take quite a while.
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
    // RCLCPP_INFO(this->get_logger(), "Set Attributes Service Called in Arborist Node");

    for (auto const & sys_attr : request->sys_attributes)
    {
      auto sys_attvalue_obj = rebet::SystemAttributeValue(sys_attr.value);
      tree.rootBlackboard()->set<rebet::SystemAttributeValue>(sys_attr.name, sys_attvalue_obj);
    }
    response->success = true;
    // RCLCPP_INFO(this->get_logger(), "Set Attributes Service finished in Arborist Node");

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
    // RCLCPP_INFO(this->get_logger(), "Set Parameter Service Called in Arborist Node");

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



  float getFloatOrNot(std::string blackboard_key)
  {
    bool gotten = false;
    float value_to_get;
    try
    {
      gotten = tree.rootBlackboard()->get<float>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      return value_to_get;
    }

    return -10.0;

  }

  int getIntOrNot(std::string blackboard_key)
  {
    bool gotten = false;
    int value_to_get;
    try
    {
      gotten = tree.rootBlackboard()->get<int>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      return value_to_get;
    }

    return -10;

  }

  std::string getFloatOrNotVector(std::string blackboard_key)
  {
    bool gotten = false;
    std::vector<double> value_to_get;
    try
    {
      gotten = tree.rootBlackboard()->get<std::vector<double>>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      std::stringstream ss;

      for (size_t i = 0; i < value_to_get.size(); ++i) {
        ss << value_to_get[i];

        if (i != value_to_get.size() - 1) {
            ss << ",";
        }
      }

      return ss.str();
    }

    return "-10.0";

  }

  float getFloatOrNotSysAtt(std::string blackboard_key)
  {
      rebet::SystemAttributeValue _value_to_get;
      bool gotten = false;
      try
      {
        gotten = tree.rootBlackboard()->get<rebet::SystemAttributeValue>(blackboard_key,_value_to_get);
      }
      catch (const std::runtime_error& error)
      {
        gotten = false;   
      }

      if(gotten)
      {
        std_msgs::msg::Float32 as_val = _value_to_get.get<rebet::SystemAttributeType::ATTRIBUTE_FLOAT>();
        return as_val.data;
      }

      return -10.0;

  }

  std::string getStringOrNot(std::string blackboard_key)
  {
    bool gotten = false;
    std::string value_to_get;
    try
    {
      gotten = tree.rootBlackboard()->get<std::string>(blackboard_key,value_to_get);
    }
    catch (const std::runtime_error& error)
    {
      gotten = false;   
    }

    if(gotten)
    {
      return value_to_get;
    }

    return blackboard_key + "not_available";

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

  template <class QR_TYPE>
  std::vector<QR_TYPE*> get_tree_qrs()
  {    
    std::vector<QR_TYPE*> qr_nodes;

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
  void get_metric_from_qrs(std::vector<QR_TYPE*> qr_nodes, std::shared_ptr<GetQR::Response>& response)
  {    
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
          qr_msg.higher_is_better = node->is_higher_better();

          response->qrs_in_tree.push_back(qr_msg);
          }
        }
  }

  template <class QR_TYPE>
  bool set_weights_of_qrs(std::vector<QR_MSG> qr_msgs, std::vector<QR_TYPE*> qr_nodes)
  {    
    std::string script = "";
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
              return false;
            }
            
            std::string weight_bb_key = (std::string)TreeNode::stripBlackboardPointer(weight->second);

            script += weight_bb_key;
            script += ":=";
            script += std::to_string(qr_msg.weight);
            script += "; ";
          }

        }

      }

      script.pop_back();
      script.pop_back(); //Removing the last '; ' as it isn't necessary.

      return inject_script_node(script);

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
      get_metric_from_qrs<SystemLevelQR>(sys_qr_nodes, response);
    }
    else
    {
      tsk_qr_nodes = get_tree_qrs<TaskLevelQR>();
      get_metric_from_qrs<TaskLevelQR>(tsk_qr_nodes, response);

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



  void handle_start_tree(const std::shared_ptr<GoalHandleBTAction> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal: Starting to tick tree");

    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<BTAction::Feedback>();
    feedback->node_status = "placeholder";

    auto result = std::make_shared<BTAction::Result>();
    int elapsed_seconds = 0;
    int time_since_last = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    NodeStatus result_of_tick;
    int total_elapsed = 0;
    int time_limit = 300;



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
      catch (const rclcpp_action::exceptions::UnknownGoalHandleError ros_error)
      {
        RCLCPP_INFO(this->get_logger(), "Tree failed because of unknown goal handle error.");
        RCLCPP_ERROR(this->get_logger(), ros_error.what());

        result_of_tick = NodeStatus::RUNNING;


      }

      feedback->node_status = toStr(result_of_tick) + " " + std::to_string(total_elapsed);
      // Publish feedback
      goal_handle->publish_feedback(feedback);
      // RCLCPP_INFO(this->get_logger(), "Publish feedback");




      if(total_elapsed >= time_limit)
      {
        result_of_tick = NodeStatus::SUCCESS;
      }
      // Check if goal is done
      if(result_of_tick != NodeStatus::RUNNING) {
        if (rclcpp::ok()) {
          auto end_message = std_msgs::msg::String();
          end_message.data = "!END!";
          publisher_->publish(end_message);
          
        
        }


      }
      else{
        //While running..
        auto curr_time_pointer = std::chrono::system_clock::now();


        int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
        int elapsed_seconds = current_time-time_since_last;
        //Every second I record an entry
        if(elapsed_seconds >= 1 )
        {
          total_elapsed++;
          RCLCPP_INFO(this->get_logger(), "one second passed");
          time_since_last = current_time;

          auto message = std_msgs::msg::String();
          std::string header = std::to_string(total_elapsed) 
          + " " + "bt_name"
          + " " + "sys_pow_metric" 
          + " " + "move_pow_metric" 
          + " " + "safety_metric" 
          + " " + "movement_efficiency" 
          + " " + "current_lighting" 
          + " " + "power_status" 
          + " " + "current_task" 
          + " " + "max_speed" 
          + " " + "obj_det_eff" 
          + " " + "obj_power" 
          + " " + "pic_rate" 
          + " " + "curr_cam_topic" 
          + " " + "pics_taken" 
          +";" ;

          std::string values = std::to_string(total_elapsed) 
          + " " + bt_name
          + " " + std::to_string(getFloatOrNot("sys_power_metric")) 
          + " " + std::to_string(getFloatOrNot("move_pow_metric")) 
          + " " + std::to_string(getFloatOrNot("safe_metric")) 
          + " " + std::to_string(getFloatOrNot("move_eff_metric")) 
          + " " + std::to_string(getFloatOrNotSysAtt("current_lighting")) 
          + " " + getStringOrNot("power_status") 
          + " " + getStringOrNot("current_task") 
          + " " + std::to_string(getFloatOrNot("max_speed"))
          + " " + getFloatOrNotVector("task_metric")
          + " " + std::to_string(getFloatOrNot("obj_power_metric"))
          + " " + std::to_string(getIntOrNot("pic_rate"))
          + " " + getStringOrNot("cam_feed")
          + " " + std::to_string(getFloatOrNot("rep_pic_take"));

          message.data = header + values;

          publisher_->publish(message);

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
  // rclcpp::Service<GetVParams>::SharedPtr _get_var_param;
  rclcpp::Service<SetWeights>::SharedPtr _set_weights;
  rclcpp::Service<SetAttributesInBlackboard>::SharedPtr _set_att_in_blackboard;
  rclcpp::Service<SetParameterInBlackboard>::SharedPtr _set_param_in_blackboard;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;


  rclcpp_action::Server<BTAction>::SharedPtr _start_tree; 
};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Arborist>();
  rclcpp::spin(node);
  rclcpp::shutdown();




  //return 0;
}
