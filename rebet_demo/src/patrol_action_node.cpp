#include "rclcpp/rclcpp.hpp"

#include "behaviortree_ros2/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/follow_waypoints.hpp"
#include "behaviortree_ros2/plugins.hpp"


namespace BT 
{
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using PoseStamped = geometry_msgs::msg::PoseStamped;



class PatrolActionNode: public RosActionNode<FollowWaypoints>
{
public:  

  //Name for the pose input port
  static constexpr const char* POSES = "poses";
  static constexpr const char* POSE_IN = "in_pose";

  PatrolActionNode(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<FollowWaypoints>(name, conf, params)
  { }


  static PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setGoal(RosActionNode::Goal& goal) override 
  {
    // goal.behavior_tree = "";

    std::vector<std::vector<double>> route =
    {
        {0.738186, -1.9545}, {1.14327, -4.02919}, {1.61136, -6.07591},
        {0.242309, -6.35477}, {-1.2673, -5.14716},
        {-3.16165, -5.15773}, {-3.78087, -6.63141}, {-4.26529, -8.46752},
        {-3.12492, -9.70488}, {-3.29297, -7.96175}, {-3.73838, -5.11978},
        {-3.70024, -2.78096}, {-3.63066, -0.465608}, {-3.48109, 1.26584},
        {-3.41194, 2.95581}, {-3.26955, 4.9035}, {-3.12751, 6.74465},
        {-3.53413, 9.17208}, {-4.79603, 9.239}, {-3.80414, 7.13158},
        {-0.169966, 6.77823}, {1.08097, 5.21487}, {1.10854, 3.86258},
        {0.968051, 1.83763}, {0.929273, -0.066145},{0.929273, -0.066145}
    };

    std::vector<geometry_msgs::msg::PoseStamped> pose_stamped_msgs;

    for (const auto& coordinates : route) {
        PoseStamped pose_stamped;
        pose_stamped.header.stamp = now();
        pose_stamped.header.frame_id = "map"; 

        pose_stamped.pose.position.x = coordinates[0];
        pose_stamped.pose.position.y = coordinates[1];
        pose_stamped.pose.position.z = 0.0;

        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;
        pose_stamped.pose.orientation.w = 1.0;

        pose_stamped_msgs.push_back(pose_stamped);
    }

    goal.poses = pose_stamped_msgs;


    return true;
  }
  

  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(logger(), "Here we are");
    RCLCPP_ERROR(logger(), "Arror: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    RCLCPP_INFO(logger(), "Current Waypoint %d", feedback->current_waypoint);
    return NodeStatus::RUNNING;
  }

private:
  int num_executions = 0;
  
};

// CreateRosNodePlugin(PatrolActionNode, "Patrol");

BT_REGISTER_ROS_NODES(factory, params)
{
    RosNodeParams aug_params;
    aug_params.nh = params.nh;
    aug_params.server_timeout = std::chrono::milliseconds(40000); //Nav2 can take a while to respond, especialy in a container.

    factory.registerNodeType<PatrolActionNode>("Patrol", aug_params);
}
}