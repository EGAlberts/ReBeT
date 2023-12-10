using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "mavros_msgs/srv/command_bool.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>




using CommandBool = mavros_msgs::srv::CommandBool;



class ArmMotors : public RosServiceNode<CommandBool>
{
public:
    static constexpr const char* ARMED = "armed_or_not";


    ArmMotors(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<CommandBool>(instance_name, conf, params)

    {}


    static PortsList providedPorts()
    {
    PortsList base_ports = RosServiceNode::providedPorts();

    PortsList child_ports = {
        InputPort<bool>(ARMED, "Whether or not to arm the motors.") 
            };

    child_ports.merge(base_ports);

    return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {      
        bool to_arm;
        getInput(ARMED, to_arm);
        request->value = to_arm;
        return true;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(node_->get_logger(), "SUCCESS IN ARM MOTORS ");

        return BT::NodeStatus::SUCCESS;
    }

};
