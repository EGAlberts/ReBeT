
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <cmath>
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "mavros_msgs/msg/state.hpp"

using State = mavros_msgs::msg::State;

class ConfirmMavrosStatus : public RosTopicSubNode<State>
{
    public:
    static constexpr const char* MODE_CNFRM = "mode_to_confirm";
    static constexpr const char* ARMD_CNFRM = "armed_to_confirm";

    
    ConfirmMavrosStatus(const std::string & instance_name,const NodeConfig &conf,const RosNodeParams& params)
        : RosTopicSubNode<State>(instance_name, conf, params)
    {
    }

    static PortsList providedPorts()
    {
        PortsList base_ports = RosTopicSubNode::providedPorts();

        PortsList child_ports = {
                        InputPort<std::string>(MODE_CNFRM),
                        InputPort<bool>(ARMD_CNFRM),

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
    BT::NodeStatus onTick(const typename State::SharedPtr& last_msg) override
    {   
        // State latest_state = *last_msg;
        bool desired_armed_status;
        std::string desired_mode;
        if(last_msg) 
        {
            getInput(ARMD_CNFRM, desired_armed_status);
            getInput(MODE_CNFRM, desired_mode);

            if(desired_armed_status == last_msg->armed && desired_mode == last_msg->mode)
            {
                RCLCPP_INFO(node_->get_logger(), "SUCCESS IN CONFIRM STATUS");

                return BT::NodeStatus::SUCCESS;
            }
            std::stringstream ss;
            ss << "Desired arm " << desired_armed_status << " armed gotten " << last_msg->armed;
            ss << "Desired mode " <<  desired_mode << " mode gotten " << last_msg->mode;
            RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        }

        RCLCPP_INFO_THROTTLE(node_->get_logger(),*node_->get_clock(), 3000, "FAILURE IN CONFIRM STATUS");

        return BT::NodeStatus::FAILURE;
    }
};