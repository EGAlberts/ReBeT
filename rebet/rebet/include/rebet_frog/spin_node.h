
#include "geometry_msgs/msg/twist.hpp"
#include "rebet/rebet_utilities.hpp"
#include "behaviortree_ros2/bt_topic_pub_node.hpp"


using Twist = geometry_msgs::msg::Twist;


class SlowlySpin : public RosTopicPubNode<Twist>
{
    public:
      const double ANGULAR_SPEED = -0.4;

    SlowlySpin(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicPubNode<Twist>(instance_name, conf, params)
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

    bool setMessage(Twist& msg) override
    {   
        std::cout << "Going to rotate now \n\n\n\n" << std::endl;
        msg.angular.z = ANGULAR_SPEED;

        return true;
    }
};