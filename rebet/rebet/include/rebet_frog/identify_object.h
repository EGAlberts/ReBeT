using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include "rebet_msgs/srv/detect_object.hpp"
#include "rebet_msgs/msg/objects_identified.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include <math.h>
#include "nav_msgs/msg/odometry.hpp"

using DetectObject = rebet_msgs::srv::DetectObject;
using Image = sensor_msgs::msg::Image;
using ObjectsIdentified = rebet_msgs::msg::ObjectsIdentified;

class IdentifyObjectService : public RosServiceNode<DetectObject>
{
public:
    static constexpr const char* IMG_IN = "in_camera_image";
    static constexpr const char* OBJ_OUT = "objs_identified";

    static constexpr const char* NUM_DETECTED = "number_detected";
    static constexpr const char* PICS_TAKEN = "pictures_taken";



    IdentifyObjectService(const std::string & instance_name,
                          const BT::NodeConfig& conf,
                          const BT::RosNodeParams& params) :
        RosServiceNode<DetectObject>(instance_name, conf, params)

    {
        num_executions = 0;
        times_anything_detected = 0;
    }


    static PortsList providedPorts()
    {
    PortsList base_ports = RosServiceNode::providedPorts();

    PortsList child_ports = { 
              InputPort<Image>(IMG_IN),
              OutputPort<ObjectsIdentified>(OBJ_OUT),
              OutputPort<std::string>("name_of_task"),
              OutputPort<int>(PICS_TAKEN),
              OutputPort<int>(NUM_DETECTED),


            };

    child_ports.merge(base_ports);

    return child_ports;
    }

    bool setRequest(typename Request::SharedPtr& request) override
    {
        RCLCPP_INFO(node_->get_logger(), "ID object request");

        setOutput("name_of_task",registrationName());
        Image camera_image;
        getInput(IMG_IN,camera_image);
        request->id = 1; //fix
        request->image = camera_image;
    
        return true;

    }

    BT::NodeStatus onFailure(ServiceNodeErrorCode error) override
    {
        RCLCPP_ERROR(node_->get_logger(), "Error: %d", error);
        return NodeStatus::FAILURE;
    }

    BT::NodeStatus onResponseReceived(const typename Response::SharedPtr& response) override
    {
        RCLCPP_INFO(node_->get_logger(), "response received");

        num_executions++;
        ObjectsIdentified obj_idd;

        obj_idd.object_names = {};
        obj_idd.probabilities = {};
        obj_idd.object_detected = false;
        
        auto probabilities = response.get()->probabilities;
        auto labels = response.get()->labels;
        
        std::stringstream res_info;
        if(labels.size() > 0) 
        {
            obj_idd.object_detected = true;
            times_anything_detected+=1;

            std::vector<std::string>::iterator iter = labels.begin();
            for(auto iter = labels.begin(); iter != labels.end(); iter++)
            {
                int index = std::distance(labels.begin(), iter);
                auto obj_label = labels[index];
                auto obj_prob = probabilities[index];
                if(obj_label == goal_object)
                {
                    times_detected++; //If there are multiple instances in boxes it'll count them too..
                }
                obj_idd.object_names.push_back(obj_label);
                obj_idd.probabilities.push_back(obj_prob);
            }
            for(unsigned int i = 0; i < obj_idd.object_names.size(); i++)
            {
            res_info << "Box: " << i << " Result: " << obj_idd.object_names[i] << " Probability: " << obj_idd.probabilities[i] << "\n";
            }
            

        }
        else
        {
        res_info << "Nothing detected" << "\n";
        }
        
        res_info << "Times " << goal_object << " has been detected: " << times_detected << "number_executions " << num_executions << "\n" ;
        RCLCPP_INFO(node_->get_logger(), res_info.str().c_str());
        
        
        // std::stringstream ss;
        // ss << "ID Result received: " << wr.result->time_elapsed << " pic rate" << wr.result->picture_rate;
        // setOutput("out_time_elapsed", wr.result->time_elapsed); 
        // setOutput(PC_RATE, wr.result->picture_rate); 
        // setOutput(DET_THRESH, wr.result->detection_threshold); 
        obj_idd.stamp = node_->now();
        setOutput(OBJ_OUT, obj_idd); 
        setOutput(PICS_TAKEN, num_executions); 
        setOutput(NUM_DETECTED, times_anything_detected); 





        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        RCLCPP_INFO(node_->get_logger(), "SUCCESS IN IDOBJ");
        return NodeStatus::SUCCESS;
    }

    private:
        int num_executions;
        std::string goal_object = "fire hydrant";//Parameterize
        int times_detected;//privatize
        int times_anything_detected;



};
