using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>

#include "suave_msgs/action/follow.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include <math.h>






using Follow = suave_msgs::action::Follow;

class FollowPipeline : public RosActionNode<Follow>
{
public:  
  const std::string PICTURE_RT_PARAM = "pic_rate";
  const std::string ACTION_SRVR = "identify_action_server"; //TODO: update

  //Name for the pose input port
  static constexpr const char* POSES = "poses";

  FollowPipeline(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<Follow>(name, conf, params)
  {
    std::cout << "Someone made me (an FollowPipeline Action Nodee)" << std::endl;

  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();

    PortsList child_ports = { 
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

    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
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
    return NodeStatus::RUNNING;
  }
private:

};
