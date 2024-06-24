#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdlib>

#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"

#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace BT;


using NavigateToPose = nav2_msgs::action::NavigateToPose;

using Point = geometry_msgs::msg::Point;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Quaternion = geometry_msgs::msg::Quaternion;
using Odometry = nav_msgs::msg::Odometry;









class VisitObstacleAction: public RosActionNode<NavigateToPose>
{
public:  

  //Name for the pose input port
  static constexpr const char* POSES = "poses";
  static constexpr const char* POSE_IN = "in_pose";

  VisitObstacleAction(const std::string& name,
                  const NodeConfig& conf,
                  const RosNodeParams& params)
    : RosActionNode<NavigateToPose>(name, conf, params)
  {
    std::cout << "Someone made me (an VisitObstacle Action Nodee)" << std::endl;
    RCLCPP_INFO(node_->get_logger(), "name of the node");
    RCLCPP_INFO(node_->get_logger(), node_->get_name());

  }

  // The specific ports of this Derived class
  // should be merged with the ports of the base class,
  // using RosActionNode::providedBasicPorts()
  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();

    PortsList child_ports = { 
              InputPort<std::vector<Point>>(POSES),
              InputPort<Odometry>(POSE_IN),
              OutputPort<float>("out_time_elapsed"),
              OutputPort<std::string>("name_of_task"),

            };

    child_ports.merge(base_ports);

    return child_ports;
  }

  // This is called when the TreeNode is ticked and it should
  // send the request to the action server

  Quaternion quaternion_from_euler_again(double ai, double aj, double ak) 
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

        double w = cj*sc - sj*cs;
        double x = cj*ss + sj*cc;
        double y = cj*cs - sj*sc;
        double z = cj*cc + sj*ss;

        Quaternion return_qua;
        return_qua.w = w;
        return_qua.x = x;
        return_qua.y = y;
        return_qua.z = z;
    
        return return_qua;
    }
  bool setGoal(RosActionNode::Goal& goal) override 
  {
    setOutput("name_of_task",registrationName());
    // #goal definition
    // geometry_msgs/PoseStamped pose
    // string behavior_tree
    std::stringstream ss;

    ss << "setGoal in pose ";



    goal.behavior_tree = "";
    Odometry odom_obj;

    getInput(POSES,poses_to_go_to_);
    getInput(POSE_IN,odom_obj);

    for (auto positi : poses_to_go_to_) {
      ss << "(";
      ss << positi.x;
      ss << ",";
      ss << positi.y;
      ss << ") ";
    }

    ss << "num_exec:" << num_executions << " ";
   

    double current_x = odom_obj.pose.pose.position.x;
    double current_y = odom_obj.pose.pose.position.y;

    double target_x = poses_to_go_to_[num_executions].x;
    double target_y = poses_to_go_to_[num_executions].y;

    double delta_y = target_y - current_y;
    double delta_x = target_x - current_x;
    
    tf2::Quaternion q_orig, q_rot, q_new;

    tf2::fromMsg(odom_obj.pose.pose.orientation, q_orig);
    // Rotate the previous pose by 180* about X
    q_rot.setRPY(0.0, 0.0, atan2(delta_y,delta_x));
    q_new = q_rot * q_orig;
    q_new.normalize();    

    goal.pose.header.frame_id = "map";

    // goal.pose.pose = odom_obj.pose.pose;


    // double magnitude = sqrt(delta_x * delta_x + delta_y * delta_y);
    // double unit_x = delta_x / magnitude;
    // double unit_y = delta_y / magnitude;

    // double offset_distance = 0.1; // 0.1 meters away from the target
    double goal_x = target_x;
    double goal_y = target_y;



    // double angle_to_target = atan2(target_y - goal_y, target_x - goal_x);
    // ss << angle_to_target;

    goal.pose.pose.orientation = tf2::toMsg(q_new);
    goal.pose.pose.position.x = goal_x;
    goal.pose.pose.position.y = goal_y;


    goal.pose.header.stamp = node_->now();



    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    


    // return true, if we were able to set the goal correctly.
    return true;
  }
  
  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult& wr) override
  {
    // #result definition
    // std_msgs/Empty result
    std::stringstream ss;
    ss << "\n\nnum exec " << num_executions << "NavigateToPose Result received: \n";
    //Unfortunately, NavigateToPose in Nav2 right now provides no actual result indicating you reached the pose or not..

    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    //If the next num_executions would exceed the size, reset to 0
    if(num_executions+1 > poses_to_go_to_.size()-1)
    {
      num_executions = 0;
    }
    else {
      num_executions++;
    }
    RCLCPP_INFO(node_->get_logger(), "SUCCESS IN RESULT RCV GOTOPOSE");

    return NodeStatus::SUCCESS;
  }

  // Callback invoked when there was an error at the level
  // of the communication between client and server.
  // This will set the status of the TreeNode to either SUCCESS or FAILURE,
  // based on the return value.
  // If not overridden, it will return FAILURE by default.
  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(node_->get_logger(), "Here we are");
    RCLCPP_ERROR(node_->get_logger(), "Arror: %d", error);
    return NodeStatus::FAILURE;
  }

  // we also support a callback for the feedback, as in
  // the original tutorial.
  // Usually, this callback should return RUNNING, but you
  // might decide, based on the value of the feedback, to abort
  // the action, and consider the TreeNode completed.
  // In that case, return SUCCESS or FAILURE.
  // The Cancel request will be send automatically to the server.
  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    // RCLCPP_INFO(node_->get_logger(), "Fdbk go to pose");

    // #feedback definition
    // geometry_msgs/PoseStamped current_pose
    // builtin_interfaces/Duration navigation_time
    // builtin_interfaces/Duration estimated_time_remaining
    // int16 number_of_recoveries
    // float32 distance_remaining

    // std::string some_text;
    // std::stringstream ss;
    // ss << "NavigateToPose Feedback received, time left: ";
    // // for (auto number : feedback->left_time) {
    // ss << feedback->estimated_time_remaining.sec;
    // // }
    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());

    
    // setOutput("objs_identified", feedback->obj_idd); 
    // setOutput(PC_RATE, feedback->picture_rate); 
    // RCLCPP_INFO(node_->get_logger(), "RUNNING IN VISIT OB FDBK");

    return NodeStatus::RUNNING;
  }
private:
  int num_executions = 0;
  std::vector<Point> poses_to_go_to_;
  
};
