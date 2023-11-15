
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>
#include "behaviortree_ros2/bt_topic_pub_node.hpp"
// Example of custom SyncActionNode (synchronous action)
// without ports.


using PoseWithCovarianceStamped = geometry_msgs::msg::PoseWithCovarianceStamped;

std::vector<double> quaternion_from_euler(double ai, double aj, double ak) 
{
    ai /= 2.0;
    aj /= 2.0;
    ak /= 2.0;
    double ci = cos(ai);
    double si = sin(ai);
    double cj = cos(aj);
    double sj = sin(aj);
    double ck = cos(ak);
    double sk = sin(ak);
    double cc = ci*ck;
    double cs = ci*sk;
    double sc = si*ck;
    double ss = si*sk;

    return {cj*sc - sj*cs, cj*ss + sj*cc, cj*cs - sj*sc, cj*cc + sj*ss};
}


class ProvideInitialPose : public RosTopicPubNode<PoseWithCovarianceStamped>
{
    public:
    

    ProvideInitialPose(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicPubNode<PoseWithCovarianceStamped>(instance_name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosTopicPubNode::providedPorts();

        PortsList child_ports = {
                };

        child_ports.merge(base_ports);

        return child_ports;
    }

    bool setMessage(PoseWithCovarianceStamped& msg) override
    {
        msg.header.frame_id = "map";
        
        msg.pose.pose.position.x = -2.0;
        msg.pose.pose.position.y = -0.5;

        msg.header.stamp = node_->now();
        std::vector<double> q = quaternion_from_euler(0, 0, 0);
        msg.pose.pose.orientation.x = q[0];
        msg.pose.pose.orientation.y = q[1];
        msg.pose.pose.orientation.z = q[2];
        msg.pose.pose.orientation.w = q[3];
        return true;
    }

};