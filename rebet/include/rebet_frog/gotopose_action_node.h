#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

using namespace BT;


using NavigateToPose = nav2_msgs::action::NavigateToPose;
using PoseStamped = geometry_msgs::msg::PoseStamped;






class GoToPoseAction: public RosActionNode<NavigateToPose>
{
public:  

  //Name for the pose input port
  static constexpr const char* POSES = "poses";

  GoToPoseAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<NavigateToPose>(name, conf, params)
  {
    std::cout << "Someone made me (an GoToPose Action Nodee)" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "name of the node");
    RCLCPP_INFO(node_->get_logger(), node_->get_name());

  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();

    PortsList child_ports = { 
              InputPort<std::vector<PoseStamped>>(POSES),
              OutputPort<float>("out_time_elapsed"),
              OutputPort<std::string>("name_of_task"),

            };

    child_ports.merge(base_ports);

    return child_ports;
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    setOutput("name_of_task",registrationName());
    // #goal definition
    // geometry_msgs/PoseStamped pose
    // string behavior_tree
    std::stringstream ss;

    ss << "setGoal in pose";
  

    goal.behavior_tree = "";
    
    getInput(POSES,poses_to_go_to_);


    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    

    goal.pose = poses_to_go_to_[num_executions];
    goal.pose.header.stamp = node_->now();


    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    // #result definition
    // std_msgs/Empty result
    std::stringstream ss;
    ss << "\n\nnum exec " << num_executions << "NavigateToPose Result received: \n";
    //Unfortunately, NavigateToPose in Nav2 right now provides no actual result indicating you reached the pose or not..

    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    //If the next num_executions would exceed the size, reset to 0
    if(num_executions+1 > poses_to_go_to_.size()-1)
    {
      num_executions = 0;
    }
    else {
      num_executions++;
    }
    RCLCPP_INFO(node_->get_logger(), "SUCCESS IN RESULT RCV GOTOPOSE");

    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(node_->get_logger(), "Here we are");
    RCLCPP_ERROR(node_->get_logger(), "Arror: %d", error);
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
    // RCLCPP_INFO(node_->get_logger(), "Fdbk go to pose");

    // #feedback definition
    // geometry_msgs/PoseStamped current_pose
    // builtin_interfaces/Duration navigation_time
    // builtin_interfaces/Duration estimated_time_remaining
    // int16 number_of_recoveries
    // float32 distance_remaining

    // std::string some_text;
    // std::stringstream ss;
    // ss << "NavigateToPose Feedback received, time left: ";
    // // for (auto number : feedback->left_time) {
    // ss << feedback->estimated_time_remaining.sec;
    // // }
    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    
    // setOutput("objs_identified", feedback->obj_idd); 
    // setOutput(PC_RATE, feedback->picture_rate); 
    // RCLCPP_INFO(node_->get_logger(), "RUNNING IN VISIT OB FDBK");

    return NodeStatus::RUNNING;
  }
private:
  int num_executions = 0;
  std::vector<PoseStamped> poses_to_go_to_;
};
