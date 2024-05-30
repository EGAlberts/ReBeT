#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"

#include "rebet_msgs/srv/set_attributes_in_blackboard.hpp"
#include "rebet_msgs/srv/set_parameter_in_blackboard.hpp"
#include "rebet_msgs/srv/get_blackboard.hpp"
#include "rebet_msgs/msg/system_attribute_value.hpp"


#include "rebet_demo/rebet_utilities.hpp"
#include "rebet_demo/system_attribute_value.hpp"
#include "rebet_demo/qr_node.h"
#include "rebet_demo/adapt_node.h"
#include "rebet_demo/patrol_qrs.h"
#include "rebet_demo/patrol_adapt.h"



#include "behaviortree_ros2/tree_execution_server.hpp"
#include "behaviortree_ros2/ros_node_params.hpp"

#include <fstream>
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>


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

  int time_since_last = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  int total_elapsed = 0;
  int time_limit = 300;

  Arborist(const rclcpp::NodeOptions& options) : TreeExecutionServer(options)
  { 
      _set_param_in_blackboard = node()->create_service<SetParameterInBlackboard>("set_parameter_in_blackboard", std::bind(&Arborist::handle_set_param_bb, this, _1, _2));
      _set_att_in_blackboard = node()->create_service<SetAttributesInBlackboard>("set_attributes_in_blackboard", std::bind(&Arborist::handle_set_atb_bb, this, _1, _2));
      _get_blackboard = node()->create_service<GetBlackboard>("get_blackboard", std::bind(&Arborist::handle_get_bb, this, _1, _2));

      // may throw ament_index_cpp::PackageNotFoundError exception
      std::string tree_dir = ament_index_cpp::get_package_share_directory("rebet_demo") + "/trees/";



      publisher_ = node()->create_publisher<std_msgs::msg::String>("arborist/reporting", 10);

      std::cout << "construction complete" << std::endl;

  }


  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override
  {
    
    auto curr_time_pointer = std::chrono::system_clock::now();


    int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
    int elapsed_seconds = current_time-time_since_last;
    //Every second I record an entry
    if(elapsed_seconds >= 1 )
    {
      RCLCPP_INFO(node()->get_logger(), "safety metric: %f %s, move power metric: %f, move quick metric: %f, max_speed: %f, detect_efficiency: %s", getFloatOrNot("safe_metric"), getStringOrNot("safe_status").c_str(), getFloatOrNot("move_pow_metric"),getFloatOrNot("move_quick_metric"), getFloatOrNot("max_speed"), getFloatOrNotVector("det_eff_metric").c_str());
      total_elapsed++;
      RCLCPP_INFO(node()->get_logger(), "%d/%d", total_elapsed, time_limit);
      time_since_last = current_time;
    }
    return std::nullopt;
  }
  
  virtual std::optional<std::string> onTreeExecutionCompleted(BT::NodeStatus status,
                                                              bool was_cancelled)
  {
    if(status == BT::NodeStatus::SUCCESS)
    {
      auto end_message = std_msgs::msg::String();
      end_message.data = "!END!";
      publisher_->publish(end_message);

      return "Ended well";
    }
    return "Ended Poorly";
  }

  void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override
  {    
    factory.registerNodeType<DetectEfficiently>("DetectEfficiently");
    
    factory.registerNodeType<MoveSafely>("MoveSafely");
    factory.registerNodeType<MovePowerEfficiently>("MovePowerEfficiently");
    factory.registerNodeType<MoveQuickly>("MoveQuickly");

    factory.registerNodeType<AdaptMaxSpeedExternal>("AdaptMaxSpeed");
    factory.registerNodeType<AdaptCameraFeedInternal>("AdaptCameraFeed");
  }

private:

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
      gotten = globalBlackboard()->get<float>(blackboard_key,value_to_get);
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
      gotten = globalBlackboard()->get<int>(blackboard_key,value_to_get);
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
      gotten = globalBlackboard()->get<std::vector<double>>(blackboard_key,value_to_get);
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
        gotten = globalBlackboard()->get<rebet::SystemAttributeValue>(blackboard_key,_value_to_get);
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
      gotten = globalBlackboard()->get<std::string>(blackboard_key,value_to_get);
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

  
  rclcpp::Service<GetBlackboard>::SharedPtr _get_blackboard;
  rclcpp::Service<SetAttributesInBlackboard>::SharedPtr _set_att_in_blackboard;
  rclcpp::Service<SetParameterInBlackboard>::SharedPtr _set_param_in_blackboard;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

};




int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<Arborist>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();

}
