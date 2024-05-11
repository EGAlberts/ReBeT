
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rebet/rebet_utilities.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "nav_msgs/msg/odometry.hpp"

using Odometry = nav_msgs::msg::Odometry;
using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;




class ProvideInitialPose : public RosTopicPubNode<PoseWithCovarianceStamped>
{
    public:
    
        static constexpr const char* POSE_IN = "in_pose";

    ProvideInitialPose(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicPubNode<PoseWithCovarianceStamped>(instance_name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosTopicPubNode::providedPorts();

        PortsList child_ports = {
                InputPort<Odometry>(POSE_IN),

                };

        child_ports.merge(base_ports);

        return child_ports;
    }

    bool setMessage(PoseWithCovarianceStamped& msg) override
    {
        Odometry odom_obj;

        auto res = getInput(POSE_IN,odom_obj);

        if(!res) { return false; }

                
        msg.header.frame_id = "map";
        
        msg.pose.pose.position.x = odom_obj.pose.pose.position.x;
        msg.pose.pose.position.y = odom_obj.pose.pose.position.y;

        msg.header.stamp = node_->now();

        msg.pose.pose.orientation = odom_obj.pose.pose.orientation;

        return true;
    }

};