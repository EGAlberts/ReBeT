
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "sensor_msgs/msg/image.hpp"

using Image = sensor_msgs::msg::Image;


class CameraFeedNode : public RosTopicSubNode<Image>
{
    public:
    
    static constexpr const char* IMG_OUT = "latest_image";

    CameraFeedNode(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicSubNode<Image>(instance_name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosTopicSubNode::providedPorts();

        PortsList child_ports = {
                        OutputPort<Image>(IMG_OUT),
                };

        child_ports.merge(base_ports);

        return child_ports;
    }

  /** Callback invoked in the tick. You must return either SUCCESS of FAILURE
   *
   * @param last_msg the latest message received since the last tick.
   * it might be empty.
   * @return the new status of the Node, based on last_msg
   */
    BT::NodeStatus onTick(const typename Image::SharedPtr& last_msg) override
    {
        if(last_msg) {
            setOutput(IMG_OUT, *last_msg);

            RCLCPP_INFO(node_->get_logger(), "SUCCESS IN CAM FEED");

            return BT::NodeStatus::SUCCESS;
            

        }
        RCLCPP_INFO(node_->get_logger(), "FAILURE IN CAM FEED");

        return BT::NodeStatus::FAILURE;
    }
};