#pragma once

#include "rebet_demo/qr_node.h"

namespace BT
{


class MoveSafely : public TaskLevelQR
{
  public:

    MoveSafely(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Safety)
    { }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(IN_LASER,"laser_scan message wrapped in a systemattributevalue instance"),
              };

      child_ports.merge(base_ports);

      return child_ports;
    }
	
    virtual void calculate_measure() override
    {
      auto is_att_present = getInputStamped(IN_LASER,_laser_attribute); 
      if(is_att_present)
      {
        auto att_timestamp = is_att_present.value();
        _laser_msg = _laser_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_LASER>();

        if(att_timestamp.time != _last_laser_timestamp.time)
        {
          _last_laser_timestamp = att_timestamp; 

          float laser_min = _laser_msg.range_min;
          float laser_max = _laser_msg.range_max;

          float nearest_object = laser_max*2; //anything above laser_max should work

          for (float const & laser_dist : _laser_msg.ranges)
          {
            if(laser_dist < laser_max && laser_dist > laser_min)
            {
              nearest_object = (laser_dist < nearest_object) ? laser_dist : nearest_object;
            }
          }

          _fitted_nearest = (nearest_object - laser_min) / (laser_max - laser_min);

          _metric = std::clamp(_fitted_nearest,0.0f,1.0f);

          if(_metric < PROXIMITY_SAFETY_THRESHOLD){
            setOutput(QR_STATUS,unsafe);
          }
          else
          {
            setOutput(QR_STATUS,safe);
          }
          output_metric();
          metric_mean();
          setOutput(MEAN_METRIC,_average_metric);
        }

      }
    }
  private:
      std::string safe = "SAFE";
      std::string unsafe = "UNSAFE";
      rebet::SystemAttributeValue _laser_attribute;
      float _fitted_nearest;

      sensor_msgs::msg::LaserScan _laser_msg;
      BT::Timestamp _last_laser_timestamp;
      const double PROXIMITY_SAFETY_THRESHOLD = 0.10;
      static constexpr const char* IN_LASER = "in_laser_scan";

};

class MovePowerEfficiently : public TaskLevelQR
{
  public:

    MovePowerEfficiently(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
    {
      _higher_is_better = false;
      _power_consumed_moving = 0.0;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(IN_ODOM,"odometry message wrapped in a systemattributevalue instance"),
              };

      child_ports.merge(base_ports);

      return child_ports;
    }

    float calculate_power_motion(float speed) {
      return 6.25 * pow(speed, 2) + 9.79 * speed + 3.66;
    }

    virtual void calculate_measure() override
    {
      auto is_att_present = getInputStamped(IN_ODOM,_odom_attribute); 

      if(is_att_present)
      {
        nav_msgs::msg::Odometry odom_msg = _odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();
        float linear_speed = hypot(fabs(odom_msg.twist.twist.linear.x), fabs(odom_msg.twist.twist.linear.y));


        auto att_timestamp = is_att_present.value();

        std::chrono::nanoseconds one_second = std::chrono::seconds(1);
        if( (att_timestamp.time-_last_odom_timestamp.time) >= one_second ) //one second has passed since last odom
        {
          std::cout << "calc measure pow att one second passed" << std::endl;

          _metric = calculate_power_motion(linear_speed);
          output_metric();
          metric_mean();
          setOutput(MEAN_METRIC,_average_metric);
          _last_odom_timestamp = att_timestamp; 
        }
      }
    }

    private:
      rebet::SystemAttributeValue _odom_attribute;
   
      Timestamp _last_odom_timestamp;

      float _power_consumed_moving;      

      static constexpr const char* IN_ODOM = "in_odom";

};  

class MoveQuickly : public TaskLevelQR
{
  public:
    builtin_interfaces::msg::Time last_timestamp;

    MoveQuickly(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::MovementEfficiency)
    { }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(IN_ODOM,"odometry message wrapped in a systemattributevalue instance"),
              };

      child_ports.merge(base_ports);

      return child_ports;
    }
	
    virtual void calculate_measure() override
    {
      auto is_att_present = getInputStamped(IN_ODOM,_odom_attribute); 

      if(is_att_present)
      {
        nav_msgs::msg::Odometry odom_msg = _odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();
        float linear_speed = hypot(fabs(odom_msg.twist.twist.linear.x), fabs(odom_msg.twist.twist.linear.y));

        auto att_timestamp = is_att_present.value();

        if(att_timestamp.time != _last_odom_timestamp.time)
        {
          if(linear_speed > 0.03) //this is to circumvent a start from stationary being considered as part of this QR.
          {
            float speed_ratio = (linear_speed/MAX_LIN_VEL);
            _metric = std::clamp(speed_ratio,0.0f,1.0f);

            output_metric();
            metric_mean();

            setOutput(MEAN_METRIC,_average_metric);
          }

          _last_odom_timestamp = att_timestamp; 
      }
      }

    }
  private:
      rebet::SystemAttributeValue _odom_attribute;

      float _linear_speed;
      Timestamp _last_odom_timestamp;


      nav_msgs::msg::Odometry _odom_msg;
      int _odom_last_timestamp_sec;

      const float MAX_LIN_VEL = 0.5;

      static constexpr const char* IN_ODOM = "in_odom";
};

class DetectEfficiently : public TaskLevelQR
{
  public:
    DetectEfficiently(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::TaskEfficiency)
    {
      _objects_visited = 0.0;
      _goal_detected = 0;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<std::vector<rebet_msgs::msg::ObjectsIdentified>>(IN_OBJ,"The objects detected through the robot's camera"),
              OutputPort<std::vector<double>>(METRIC, "To what extent is this property fulfilled"),
              OutputPort<std::vector<double>>(MEAN_METRIC, "To what extent is this property fulfilled on average"),};
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      if(process_task_progress())
      {
        std::vector<double> metrics = {(double)_goal_detected, _objects_visited};
        setOutput(METRIC,metrics); 
        //TODO: add average
      }
    }

    bool process_task_progress() //Filling the buffer/batch
    {
      std::vector<rebet_msgs::msg::ObjectsIdentified> objects_msg_vec;
      setOutput(QR_STATUS,not_detected); //default

      auto is_att_present = getInputStamped(IN_OBJ, objects_msg_vec);

      if(is_att_present)
      {
        auto att_timestamp = is_att_present.value();

        if(att_timestamp.time != _last_timestamp.time)
        {
          for (const auto& obj_msg : objects_msg_vec) 
          {
            if(obj_msg.object_detected)
            {
              setOutput(QR_STATUS,detected);
              for (const auto& object_name : obj_msg.object_names)
              {
                if(object_name == goal_object){
                  _goal_detected++;
                }
              }
            }
          }
          _objects_visited+=1.0;
          _last_timestamp = att_timestamp;

          return true;
        }
      }
      return false;
    }

    private:
      std::string detected = "DETECTED";
      std::string not_detected = "NOT_DETECTED";
      std::string goal_object = "fire hydrant";

      int _goal_detected;
      Timestamp _last_timestamp;
      double _objects_visited;
      static constexpr const char* IN_OBJ = "in_object_identified";
};

}   // namespace BT
