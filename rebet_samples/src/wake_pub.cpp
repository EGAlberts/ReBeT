#include "behaviortree_ros2/bt_topic_pub_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "std_msgs/msg/string.hpp"

namespace BT {

using String = std_msgs::msg::String;

class Wake : public RosTopicPubNode<String>
{
    public:
    
    Wake(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicPubNode<String>(instance_name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        return RosTopicPubNode::providedPorts();
    }

    bool setMessage(String& msg) override
    {                
        msg.data = "Hi! I'm awake :)";
        return true;
    }

};
CreateRosNodePlugin(Wake, "Wake");
}