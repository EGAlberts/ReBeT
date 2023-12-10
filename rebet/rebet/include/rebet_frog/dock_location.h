using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "nav_msgs/srv/get_map.hpp"
#include <math.h>

#include "rebet/rebet_utilities.hpp"

using PoseStamped = geometry_msgs::msg::PoseStamped;


class SetDockLocation : public BT::SyncActionNode
{
public:
    static constexpr const char* DOCK_POS = "dock_position";

    SetDockLocation(const std::string & instance_name,
                          const BT::NodeConfig& conf) :
        BT::SyncActionNode(instance_name, conf)

    {}


    static PortsList providedPorts()
    {
    PortsList child_ports = { 
                OutputPort<std::vector<PoseStamped>>(DOCK_POS),
            };

    return child_ports;
    }

    BT::NodeStatus tick() override
    {
        std::vector<PoseStamped> poses;

        PoseStamped msg;
        msg.header.frame_id = "map";
        
        msg.pose.position.x = DOCK_X;
        msg.pose.position.y = DOCK_Y;

        std::vector<double> q = quaternion_from_euler(0, 0, 0);
        msg.pose.orientation.x = q[0];
        msg.pose.orientation.y = q[1];
        msg.pose.orientation.z = q[2];
        msg.pose.orientation.w = q[3];
        poses.push_back(msg);
     
        setOutput(DOCK_POS, poses);
        
        return BT::NodeStatus::SUCCESS;
    }

private:
    const double DOCK_X = -2.0;
    const double DOCK_Y = 2.0;

    

};
