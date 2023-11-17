#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rebet_msgs/action/identify_object.hpp"
#include "rebet_msgs/msg/variable_parameter.hpp"

#include "rebet_msgs/msg/objects_identified.hpp"
#include "nav_msgs/srv/get_map.hpp"
#include "rebet/variable_action_node.h"
#include "rclcpp/parameter_value.hpp"
#include "darknet_ros_msgs/action/check_for_objects.hpp"
#include "sensor_msgs/msg/image.hpp"


using namespace BT;

using Image = sensor_msgs::msg::Image;
using IDObject = darknet_ros_msgs::action::CheckForObjects;
using ObjectsIdentified = rebet_msgs::msg::ObjectsIdentified;

//checkForObjectsActionName
class IdentifyObjectAction: public VariableActionNode<IDObject>
{
public:
  static constexpr const char* DET_THRESH = "out_det_threshold";
  static constexpr const char* PC_RATE = "out_picture_rate";
  static constexpr const char* IMG_IN = "in_camera_image";
  static constexpr const char* OBJ_OUT = "objs_identified";


  
  const std::string PICTURE_RT_PARAM = "pic_rate";
  const std::string ACTION_SRVR = "detectobject_action_server";

  std::string goal_object;//privatize
  int times_detected;//privatize

  IdentifyObjectAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : VariableActionNode<IDObject>(name, conf, params)
  {
    std::cout << "Someone made me (an IdentifyObject Action Nodee)" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "name of the node");
    RCLCPP_INFO(node_->get_logger(), node_->get_name());

    goal_object = "fire_hydrant"; //Parameterize
    times_detected = 0;
    // _var_params.server_name = ACTION_SRVR;

    std::vector<int> pc_rate_values{1, 3, 5, 7};
    registerAdaptations<int>(pc_rate_values, PICTURE_RT_PARAM, ACTION_SRVR);
    
    setOutput(VARIABLE_PARAMS, _var_params);
    num_executions = 0;
  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    PortsList base_ports = VariableActionNode::providedPorts();

    PortsList child_ports = { 
              InputPort<Image>(IMG_IN),
              OutputPort<ObjectsIdentified>(OBJ_OUT)
              // OutputPort<float>("out_time_elapsed"),
              // OutputPort<int32_t>(PC_RATE),
              // OutputPort<int32_t>(DET_THRESH) 

            };

    child_ports.merge(base_ports);

    return child_ports;
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(VariableActionNode::Goal& goal) override 
  {
    Image camera_image;
    getInput(IMG_IN,camera_image);
    goal.id = 1; //fix
    goal.image = camera_image;
    
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    num_executions++;
    ObjectsIdentified obj_idd;

    obj_idd.object_names = {};
    obj_idd.probabilities = {};
    obj_idd.object_detected = false;
    
    
    std::stringstream res_info;
    if(wr.result->bounding_boxes.bounding_boxes.size() > 0) 
    {
        obj_idd.object_detected = true;

        for(auto & box : wr.result->bounding_boxes.bounding_boxes)
        {
          if(box.class_id == goal_object)
          {
            times_detected++; //If there are multiple instances in boxes it'll count them too..
          }
          obj_idd.object_names.push_back(box.class_id);
          obj_idd.probabilities.push_back(box.probability);
        }
        for(unsigned int i = 0; i < obj_idd.object_names.size(); i++)
        {
          res_info << "Box: " << i << " Result: " << obj_idd.object_names[i] << " Probability: " << obj_idd.probabilities[i] << "\n";
        }
        

    }
    else
    {
      res_info << "Nothing detected" << "\n";
    }
    
    res_info << "Times " << goal_object << " has been detected: " << times_detected << "number_executions " << num_executions << "\n" ;
    RCLCPP_INFO(node_->get_logger(), res_info.str().c_str());
    
    
    // std::stringstream ss;
    // ss << "ID Result received: " << wr.result->time_elapsed << " pic rate" << wr.result->picture_rate;
    // setOutput("out_time_elapsed", wr.result->time_elapsed); 
    // setOutput(PC_RATE, wr.result->picture_rate); 
    // setOutput(DET_THRESH, wr.result->detection_threshold); 
    obj_idd.stamp = node_->now();
    setOutput(OBJ_OUT, obj_idd); 



    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    RCLCPP_INFO(node_->get_logger(), "SUCCESS IN IDOBJ");
    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    //No feedback.

    // std::string some_text;
    // std::stringstream ss;
    // ss << "ID Feedback received obj detected: ";
    // // for (auto number : feedback->left_time) {
    // ss << feedback->obj_idd.object_detected;
    // // }
    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    // //getInput("rb_name", some_text);
    // std::stringstream sstwo;
    
    // setOutput("objs_identified", feedback->obj_idd); 
    // setOutput(PC_RATE, feedback->picture_rate); 

    // //of note is that miraculously the BT CPP knows to send this to the current_position field of the blackboard, which is picked up in an QR, because it is specified in the BT xml
    
    // sstwo << "Port info received: ";
    // // for (auto number : feedback->left_time) {
    // sstwo << some_text;
    // // }
    // RCLCPP_INFO(node_->get_logger(), sstwo.str().c_str());

    RCLCPP_INFO(node_->get_logger(), "FEEDBACK RUNNING IN IDOBJ");

    return NodeStatus::RUNNING;
  }
  private:
    int num_executions;
};
