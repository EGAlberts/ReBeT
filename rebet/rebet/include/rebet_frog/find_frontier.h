using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "wavefront_frontier_msgs/srv/get_furthest_frontier.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>
#include "nav_msgs/msg/odometry.hpp"

using Odometry = nav_msgs::msg::Odometry;
using PoseStamped = geometry_msgs::msg::PoseStamped;


class FindFrontier : public RosServiceNode<wavefront_frontier_msgs::srv::GetFurthestFrontier>
{
public:
    static constexpr const char* OBS_POS = "obstacle_positions";
    static constexpr const char* OBS_NUM = "obstacle_number";

    FindFrontier(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<wavefront_frontier_msgs::srv::GetFurthestFrontier>(instance_name, conf, params)

    {}


    static PortsList providedPorts()
    {
    PortsList base_ports = RosServiceNode::providedPorts();

    PortsList child_ports = { 
                InputPort<Odometry>(IN_ODOM,"odometry message "),
                InputPort<nav_msgs::msg::OccupancyGrid>(IN_MAP,"the map"),
                OutputPort<std::vector<PoseStamped>>(OUT_FRONT,"the frontier"),
            };

    child_ports.merge(base_ports);

    return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {

        RCLCPP_INFO(node_->get_logger(), "Find frontier request");

        // nav_msgs/OccupancyGrid occupancy_grid
        // geometry_msgs/PoseStamped current_pose
        begin_wait_ = node_->now();
        auto odom_res = getInput(IN_ODOM,_odom_msg); 
        auto map_res = getInput(IN_MAP,_occupancy_grid);
        auto const timeout = rclcpp::Duration::from_seconds( double(wait_timeout_.count()) / 1000);
        while(!(odom_res && map_res))
        {
            odom_res = getInput(IN_ODOM,_odom_msg); 
            map_res = getInput(IN_MAP,_occupancy_grid);
            RCLCPP_INFO(node_->get_logger(), "Blocking untl receiving both map and odom");
            RCLCPP_INFO(node_->get_logger(), "Elapsed time: %f seconds", (node_->now() - begin_wait_).seconds());
            if((node_->now() - begin_wait_) > timeout )
            {
                RCLCPP_INFO(node_->get_logger(), "Timed out waiting for both map and odom to be ready");
                std::cout << "odom_res was " << (bool)odom_res << " map_res was " << (bool)map_res << std::endl;

                return false;
            }
        }
        if(odom_res && map_res)
        {
            request->occupancy_grid = _occupancy_grid;
            
            request->current_pose = _odom_msg.pose.pose;

            return true;
        }

        return false;

    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(node_->get_logger(), "Response received find frontier");

        auto frontier_location = response.get()->frontier_location;
        bool location_found = response.get()->frontiers_found;

        if(!location_found)
        {
            return BT::NodeStatus::FAILURE;
        }
        
        std::vector<geometry_msgs::msg::PoseStamped> frontier_poses;
        frontier_poses.push_back(frontier_location);

        setOutput(OUT_FRONT,frontier_poses);
        return BT::NodeStatus::SUCCESS;
    }

    private:
       static constexpr const char* IN_ODOM = "in_odom";
       static constexpr const char* IN_MAP = "map_to_find_frontier";
       static constexpr const char* OUT_FRONT = "frontier_positions";
        std::chrono::milliseconds wait_timeout_ = std::chrono::milliseconds(10000);
       Odometry _odom_msg;
       nav_msgs::msg::OccupancyGrid _occupancy_grid;
       rclcpp::Time begin_wait_;


};
