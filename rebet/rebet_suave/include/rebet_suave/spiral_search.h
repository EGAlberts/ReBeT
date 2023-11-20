using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>




using ChangeState = lifecycle_msgs::srv::ChangeState;



class SpiralSearch : public RosServiceNode<ChangeState>
{
public:
    static constexpr const char* DES_MODE = "desired_mode";

    SpiralSearch(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<ChangeState>(instance_name, conf, params)

    {}


    static PortsList providedPorts()
    {
    PortsList base_ports = RosServiceNode::providedPorts();

    PortsList child_ports = { 
            };

    child_ports.merge(base_ports);

    return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        request->transition.id = lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE;
        return true;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        bool started_successfully = response.get()->success;

        if(started_successfully)
        {
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::FAILURE;
        }
    }

};
