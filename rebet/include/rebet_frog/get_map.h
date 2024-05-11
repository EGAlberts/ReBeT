using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "nav_msgs/srv/get_map.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>


using PoseStamped = geometry_msgs::msg::PoseStamped;



class GetMap : public RosServiceNode<nav_msgs::srv::GetMap>
{
public:
    static constexpr const char* THE_MAP = "map_retrieved";

    GetMap(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<nav_msgs::srv::GetMap>(instance_name, conf, params)

    {}


    static PortsList providedPorts()
    {
    PortsList base_ports = RosServiceNode::providedPorts();

    PortsList child_ports = { 
                OutputPort<nav_msgs::msg::OccupancyGrid>(THE_MAP),
            };

    child_ports.merge(base_ports);

    return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        RCLCPP_INFO(node_->get_logger(), "Get map set");

        return true;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(node_->get_logger(), "Get map response");

        auto mission_map = response.get()->map;

        setOutput(THE_MAP, mission_map);

        return BT::NodeStatus::SUCCESS;
    }

    private:


};
