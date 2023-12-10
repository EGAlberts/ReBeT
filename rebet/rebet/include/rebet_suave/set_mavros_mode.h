using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "mavros_msgs/srv/set_mode.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>




using SetMode = mavros_msgs::srv::SetMode;



class SetMavrosMode : public RosServiceNode<SetMode>
{
public:
    static constexpr const char* DES_MODE = "desired_mode";
    static constexpr const char* OBS_NUM = "obstacle_number";

    SetMavrosMode(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<SetMode>(instance_name, conf, params)

    {}


    static PortsList providedPorts()
    {
    PortsList base_ports = RosServiceNode::providedPorts();

    PortsList child_ports = { 
        InputPort<std::string>(DES_MODE, "which mode do you want mavros to be in e.g. guided")
            };

    child_ports.merge(base_ports);

    return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        std::string port_mode;
        getInput(DES_MODE, port_mode);
        request->custom_mode = port_mode;
        return true;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(node_->get_logger(), "SUCCESS IN MAVROS MODE");

        return BT::NodeStatus::SUCCESS;
    }

};
