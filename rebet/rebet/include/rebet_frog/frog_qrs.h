#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rebet_msgs/msg/objects_identified.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rebet/qr_node.h"


namespace BT
{
/**
 * @brief The QRNode is used to constrain variable action nodes it decorates.
 *
 *
 * 
 *
 * 
 * 
 * 
 * 
 * 
 * Note: If in the future a BT should be designed with multiple instances of the same QR, it would become impractical. Right now the only feasible case it having two QRs which are never active simultaneously. 
 * In all other cases it would require creating a new (very similar) subclass as the linked blackboard entries would otherwise cause issue.
 */

class ObjectDetectionEfficiencyQR : public QRNode
{
  public:
    ObjectDetectionEfficiencyQR(const std::string& name, const NodeConfig& config) : QRNode(name, config)
    {
      _counter = 0;

      _last_timestamp = builtin_interfaces::msg::Time();
      num_obj_det_curr_exec = 0;
      pictures_taken = 0.0;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = QRNode::providedPorts();

      PortsList child_ports =  {
              InputPort<rebet_msgs::msg::ObjectsIdentified>(IN_OBJ,"The objects detected through the robot's camera")};
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      _counter += 1;

      _metric = std::clamp((pictures_taken / MAX_DETECTABLE),0.0,1.0);
      num_obj_det_curr_exec = 0;
      pictures_taken = 0.0;
      output_metric();
      metric_mean();
      setOutput(MEAN_METRIC,_average_metric);
    }

    void process_task_progress()
    {
      rebet_msgs::msg::ObjectsIdentified objects_msg;

      getInput(IN_OBJ, objects_msg);

      if((objects_msg.object_detected == true) && (objects_msg.stamp != _last_timestamp))
      {
        num_obj_det_curr_exec+=objects_msg.object_names.size();
        pictures_taken+=1.0;
        _last_timestamp = objects_msg.stamp;
      }

    }

    virtual NodeStatus tick() override
    {
      setStatus(NodeStatus::RUNNING);
      const NodeStatus child_status = child_node_->executeTick();

      switch (child_status)
      {
        case NodeStatus::SUCCESS: {
          calculate_measure(); //Only after the task below has finished
          resetChild();
          return NodeStatus::SUCCESS;
        }

        case NodeStatus::FAILURE: {
          resetChild();
          return NodeStatus::FAILURE;
        }

        case NodeStatus::RUNNING: {
          process_task_progress();
          return NodeStatus::RUNNING;
        }

        case NodeStatus::SKIPPED: {
          return NodeStatus::SKIPPED;
        }
        case NodeStatus::IDLE: {
          throw LogicError("[", name(), "]: A child should not return IDLE");
        }
      }
      return status();

    }

    private:
      int num_obj_det_curr_exec;
      builtin_interfaces::msg::Time _last_timestamp;
      const double MAX_DETECTABLE = 7.0; //Corresponds to the max picture rate assuming one object is detected per picture on average..
      double pictures_taken;
      int _counter;
      static constexpr const char* IN_OBJ = "objs_identified";
      static constexpr const char* PIC_RATE = "current_pic_rate";

};

class PowerQR : public QRNode
{
  public:
    int counter;
    double detection_threshold; 
    std::string goal_object;
    double times_detected;
    builtin_interfaces::msg::Time last_timestamp;
    double max_detected;
    int detected_in_window;
    int window_start;
    PowerQR(const std::string& name, const NodeConfig& config) : QRNode(name, config)
    {
      counter = 0;

      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      _last_odom = _window_start;
      _obj_last_timestamp = builtin_interfaces::msg::Time();
      _odom_last_timestamp_sec = 0;
      
      _motion_consumption = 0.0;
      _idle_consumption = 0.0;
      _laser_consumption = 0.0;

      _pictures_taken_in_window = 0;


      
    }

    void initialize(float max_pics_ps, int window_length)
    {
        _max_pics_ps = max_pics_ps;
        _window_length = window_length;

        _max_motion_consumption =  ((6.25 * pow(WAFFLE_MAX_LIN_VEL, 2.0) + 9.79 * WAFFLE_MAX_LIN_VEL +3.66)) * (_window_length+1);
        _max_picture_consumption = (max_pics_ps * (_window_length+1)) * DETECTION_AVG_POW;
        _max_idle_consumption = IDLE_POW * (_window_length + 1);
        _max_laser_consumption = LASER_POW * (_window_length + 1);

        _max_consumption = _max_motion_consumption + _max_picture_consumption + _max_idle_consumption +  _max_laser_consumption;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = QRNode::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(IN_ODOM,"odometry message wrapped in a systemattributevalue instance"),
              InputPort<int32_t>(IN_PIC_RATE,"picture rate of the ID child action"),
              InputPort<rebet_msgs::msg::ObjectsIdentified>(IN_OBJ,"The objects detected through the robot's camera")
              };

      child_ports.merge(base_ports);

      return child_ports;
    }

    static float calculate_power_motion(float speed) {
      return 6.25 * pow(speed, 2) + 9.79 * speed + 3.66;
    }
	
    virtual void calculate_measure() override
    {
      auto res = getInput(IN_ODOM,_odom_attribute); 
      auto pic_res = getInput(IN_PIC_RATE,_pic_rate);

      auto object_res = getInput(IN_OBJ, _objects_msg);

      if(res)
      {
        _odom_msg = _odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();
        _linear_speed = hypot(fabs(_odom_msg.twist.twist.linear.x), fabs(_odom_msg.twist.twist.linear.y));
      }

      if(object_res)
      {
        if(pic_res)
        {
          if(_objects_msg.stamp != _obj_last_timestamp)
          {
            std::cout << "added to pictures taken in window " << _pic_rate << std::endl;
            _pictures_taken_in_window += _pic_rate;
            _obj_last_timestamp = _objects_msg.stamp;
          }
        }
      }


      if( (_odom_msg.header.stamp.sec-_odom_last_timestamp_sec) >= 1 ) //one second has passed since last odom
      {
        
        _motion_consumption+= calculate_power_motion(_linear_speed);
        _laser_consumption+= LASER_POW;
        _idle_consumption+=  IDLE_POW;

        _odom_last_timestamp_sec = _odom_msg.header.stamp.sec;
        
      }



      auto curr_time_pointer = std::chrono::system_clock::now();

      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;

      //std::cout << "elapsed seconds and window length inside powerQR " << elapsed_seconds << " " << _window_length << std::endl;


      if(elapsed_seconds >= _window_length)
      {
        float picture_consumption = _pictures_taken_in_window * DETECTION_AVG_POW;
        std::cout << "_pictures_taken_in_window" << _pictures_taken_in_window << std::endl;
        std::cout << "_motion_consumption " << _motion_consumption << std::endl;
        std::cout << "picture_consumption " << picture_consumption  << std::endl;
        std::cout << "_idle_consumption " << _idle_consumption << std::endl;
        std::cout << "_laser_consumption " << _laser_consumption << std::endl;


        std::vector<float> consumption_ratios = {(_motion_consumption/_max_motion_consumption), (picture_consumption/_max_picture_consumption), (_idle_consumption/_max_idle_consumption), (_laser_consumption/_max_laser_consumption)};
        std::cout << "Power consumption of pic" << consumption_ratios[1] << std::endl;

        int non_zero_ratios = 0;
        float non_zero_sum = 0.0; 
        for (float const & consumption_ratio : consumption_ratios)
        {
          std::cout << "conratio " << consumption_ratio << std::endl;
          if(consumption_ratio != 0.0f)
          {
            non_zero_ratios++;
            non_zero_sum+=consumption_ratio;
          }
        }
        
    
        //std::cout << total_consumption << " is ttcnsmp " <<  _max_consumption << " is max consumption" << std::endl;

        _metric = 1.0f - std::clamp((non_zero_sum/non_zero_ratios),0.0f,1.0f); //1 - because power consumption is a bad thing for the QA

        output_metric();
        metric_mean();

        setOutput(MEAN_METRIC,_average_metric);

        _window_start = current_time;
        _motion_consumption = 0.0;
        _laser_consumption = 0.0;
        _idle_consumption = 0.0;

        _pictures_taken_in_window = 0;
      }

    }
  private:
      float _max_pics_ps;
      rebet::SystemAttributeValue _odom_attribute;
      int32_t _pic_rate;
      int _pictures_taken_in_window;
      float _motion_consumption;
      float _max_consumption;
      float _max_motion_consumption;
      float _max_picture_consumption;
      float _max_picture_taken;
      float _idle_consumption;
      float _max_idle_consumption;
      float _laser_consumption;
      float _max_laser_consumption;
      int _window_length;
      int _window_start;
      int _last_odom;
      float _linear_speed;

      nav_msgs::msg::Odometry _odom_msg;
      rebet_msgs::msg::ObjectsIdentified _objects_msg;
      int _odom_last_timestamp_sec;
      builtin_interfaces::msg::Time _obj_last_timestamp;


      const float IDLE_POW = 1.14; //as caused by the detection software running. Idle consumption of the robot as a whole is considered a constant factor.
      const float DETECTION_AVG_POW = 10.919800; //watts
      const float LASER_POW = 2.34; //watts
      const float WAFFLE_MAX_LIN_VEL = 0.26;


      static constexpr const char* IN_PIC_RATE = "in_picture_rate";
      static constexpr const char* IN_ODOM = "in_odom";
      static constexpr const char* IN_OBJ = "objs_identified";
};

class MovementEfficiencyQR : public QRNode
{
  public:
    builtin_interfaces::msg::Time last_timestamp;
    int window_start;

    MovementEfficiencyQR(const std::string& name, const NodeConfig& config) : QRNode(name, config)
    {
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      _last_odom = _window_start;
      _odom_last_timestamp = builtin_interfaces::msg::Time();
    }

    void initialize(int window_length)
    {
        _window_length = window_length;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = QRNode::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(IN_ODOM,"odometry message wrapped in a systemattributevalue instance"),
              };

      child_ports.merge(base_ports);

      return child_ports;
    }
	
    virtual void calculate_measure() override
    {
      auto res = getInput(IN_ODOM,_odom_attribute); 

      if(res)
      {
        _odom_msg = _odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();


        if(_odom_msg.header.stamp != _odom_last_timestamp)
        {
          _linear_speed = hypot(fabs(_odom_msg.twist.twist.linear.x), fabs(_odom_msg.twist.twist.linear.y));
          _odom_last_timestamp = _odom_msg.header.stamp;

          float speed_ratio = (_linear_speed/WAFFLE_MAX_LIN_VEL);
          _metric = std::clamp(speed_ratio,0.0f,1.0f);

          output_metric();
          metric_mean();

          setOutput(MEAN_METRIC,_average_metric);
        }
      }

      // auto curr_time_pointer = std::chrono::system_clock::now();

      // int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      // int elapsed_seconds = current_time-_window_start;

      // if(elapsed_seconds >= _window_length)
      // {
      //   _window_start = current_time;
      // }

    }
  private:
      rebet::SystemAttributeValue _odom_attribute;
      int _window_length;
      int _window_start;
      int _last_odom;
      float _linear_speed;
      builtin_interfaces::msg::Time _odom_last_timestamp;


      nav_msgs::msg::Odometry _odom_msg;
      int _odom_last_timestamp_sec;

      const float WAFFLE_MAX_LIN_VEL = 0.26;

      static constexpr const char* IN_ODOM = "in_odom";
};

class SafetyQR : public QRNode
{
  public:
    int counter;
    double detection_threshold; 
    std::string goal_object;
    double times_detected;
    builtin_interfaces::msg::Time last_timestamp;
    double max_detected;
    int detected_in_window;
    int window_start;
    SafetyQR(const std::string& name, const NodeConfig& config) : QRNode(name, config)
    {
      counter = 0;

      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
      _obj_last_timestamp = builtin_interfaces::msg::Time();
      _odom_last_timestamp_sec = 0;
            
    }

    // void initialize(int window_length)
    // {

    // }


    static PortsList providedPorts()
    {
      PortsList base_ports = QRNode::providedPorts();

      PortsList child_ports = { 
              InputPort<rebet::SystemAttributeValue>(IN_LASER,"laser_scan message wrapped in a systemattributevalue instance"),
              };

      child_ports.merge(base_ports);

      return child_ports;
    }
	
    virtual void calculate_measure() override
    {
      auto res = getInput(IN_LASER,_laser_attribute); 

      if(res)
      {
        _laser_msg = _laser_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_LASER>();

        if(_laser_msg.header.stamp != _obj_last_timestamp)
        {
          _obj_last_timestamp = _laser_msg.header.stamp; 

          //The _laser_msg is certified fresh

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
        }

      }


      
      _metric = std::clamp(_fitted_nearest,0.0f,1.0f);

      auto curr_time_pointer = std::chrono::system_clock::now();

      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;

      // if(elapsed_seconds >= _window_length)
      // {
      output_metric();
      metric_mean();

      setOutput(MEAN_METRIC,_average_metric);

      _window_start = current_time;
      // }

    }
  private:
      rebet::SystemAttributeValue _laser_attribute;
      int _window_length;
      int _window_start;
      float _fitted_nearest;

      sensor_msgs::msg::LaserScan _laser_msg;
      int _odom_last_timestamp_sec;
      builtin_interfaces::msg::Time _obj_last_timestamp;

      static constexpr const char* IN_LASER = "in_laser_scan";

};


}   // namespace BT
