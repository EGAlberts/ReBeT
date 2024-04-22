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

#include "rebet_frog/frog_constants.hpp"

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

class ObjectDetectionEfficiencyQR : public TaskLevelQR
{
  public:
    ObjectDetectionEfficiencyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::TaskEfficiency)
    {
      _counter = 0;

      _last_timestamp = builtin_interfaces::msg::Time();
      num_obj_det_curr_exec = 0;
      pictures_taken = 0.0;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<std::vector<rebet_msgs::msg::ObjectsIdentified>>(IN_OBJ,"The objects detected through the robot's camera")};
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override //process the buffer/batch
    {
      _counter += 1;

      _metric = std::clamp((pictures_taken / MAX_DETECTABLE),0.0,1.0);
      num_obj_det_curr_exec = 0;
      pictures_taken = 0.0;
      output_metric();
      metric_mean();
      setOutput(MEAN_METRIC,_average_metric);
    }

    void process_task_progress() //Filling the buffer/batch
    {
      std::vector<rebet_msgs::msg::ObjectsIdentified> objects_msg_vec;
      setOutput(QR_STATE,not_detected); //default


      getInput(IN_OBJ, objects_msg_vec);

      if(objects_msg_vec[0].stamp != _last_timestamp)
      {
        for (const auto& obj_msg : objects_msg_vec) 
        {
          if(obj_msg.object_detected)
          {
            setOutput(QR_STATE,detected);
          }

          num_obj_det_curr_exec+=obj_msg.object_names.size();

        }
        pictures_taken+=(double)objects_msg_vec.size();
        _last_timestamp = objects_msg_vec[0].stamp;
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
      std::string detected = std::string(OBJECT_DETECTED_STRING);
      std::string not_detected = std::string(OBJECT_NOT_DETECTED_STRING);


      int num_obj_det_curr_exec;
      builtin_interfaces::msg::Time _last_timestamp;
      const double MAX_DETECTABLE = 7.0; //Corresponds to the max picture rate assuming one object is detected per picture on average..
      double pictures_taken;
      int _counter;
      static constexpr const char* IN_OBJ = "objs_identified";
      static constexpr const char* PIC_RATE = "current_pic_rate";

};


class ObjectDetectionPowerQR : public TaskLevelQR
{
  public:

    ObjectDetectionPowerQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
    {
      _obj_last_timestamp = builtin_interfaces::msg::Time();

      _higher_is_better = false;
      pictures_taken = 0.0;      
    }

    void process_task_progress()
    {
      std::vector<rebet_msgs::msg::ObjectsIdentified> objects_msg_vec;

      getInput(IN_OBJ, objects_msg_vec);


      if(objects_msg_vec[0].stamp != _obj_last_timestamp)
      {
        for (const auto& obj_msg : objects_msg_vec) 
        {
          pictures_taken+=(double)objects_msg_vec.size();
        }
        _obj_last_timestamp = objects_msg_vec[0].stamp;
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


    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {
              InputPort<std::vector<rebet_msgs::msg::ObjectsIdentified>>(IN_OBJ,"The objects detected through the robot's camera")};
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      _metric = pictures_taken * DETECTION_AVG_POW;
      pictures_taken = 0.0;
      output_metric();
      metric_mean();
      setOutput(MEAN_METRIC,_average_metric);
     
      

    }
  private:
      float _max_pics_ps;
      int32_t _pic_rate;
      double pictures_taken;
      float _max_picture_consumption;
      float _max_picture_taken;

      int _window_length;
      int _window_start;

      builtin_interfaces::msg::Time _obj_last_timestamp;


      



      static constexpr const char* IN_PIC_RATE = "in_picture_rate";
      static constexpr const char* IN_OBJ = "objs_identified";
};


class MovementPowerQR : public TaskLevelQR
{
  public:

    MovementPowerQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Power)
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

    static float calculate_power_motion(float speed) {
      return 6.25 * pow(speed, 2) + 9.79 * speed + 3.66;
    }

    void process_movement_progress()
    {
      auto res = getInput(IN_ODOM,_odom_attribute); 

      if(res)
      {
        nav_msgs::msg::Odometry odom_msg = _odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();
        float linear_speed = hypot(fabs(odom_msg.twist.twist.linear.x), fabs(odom_msg.twist.twist.linear.y));


        if( (odom_msg.header.stamp.sec-_odom_last_timestamp_sec) >= 1 ) //one second has passed since last odom
        {
          _power_consumed_moving+= calculate_power_motion(linear_speed);
          _odom_last_timestamp_sec = odom_msg.header.stamp.sec; 
        }
      }

    }

    virtual void calculate_measure() override
    {
          _metric = _power_consumed_moving;
          _power_consumed_moving = 0.0;
          output_metric();
          metric_mean();
          setOutput(MEAN_METRIC,_average_metric);
          std::cout << "metric in  mov power QR " << _metric << std::endl;

          
      
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
          process_movement_progress();
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
      rebet::SystemAttributeValue _odom_attribute;
   
      int _odom_last_timestamp_sec;

      float _power_consumed_moving;      

      static constexpr const char* IN_ODOM = "in_odom";

};  




class SimpleSystemPowerQR : public SystemLevelQR
{
  public:

    SimpleSystemPowerQR(const std::string& name, const NodeConfig& config) : SystemLevelQR(name, config, QualityAttribute::Power)
    {
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      _idle_consumption = 0.0;
      _laser_consumption = 0.0; 
      _cumulative_consumption = 0.0;

      _higher_is_better = false;

    }


    void initialize(float max_pics_ps, int window_length)
    {
        _window_length = window_length;

          // _max_idle_consumption = IDLE_POW * (_window_length + 1);
          // _max_laser_consumption = LASER_POW * (_window_length + 1);

    }


    static PortsList providedPorts()
    {
      PortsList base_ports = SystemLevelQR::providedPorts();

      PortsList child_ports = { 
              };

      child_ports.merge(base_ports);

      return child_ports;
    }

    
	
    virtual void calculate_measure() override
    {

      system_power_consumption();

      _metric = _laser_consumption + _idle_consumption;

      _cumulative_consumption+= _metric;

      output_metric();

      metric_mean();

      setOutput(MEAN_METRIC,_average_metric);

      if(_cumulative_consumption > MIN_CHARGE_THRESHOLD)
      {
        _cumulative_consumption = 0.0;
        setOutput(QR_STATE,"below_min"); //it is set back to OK in the BT elsewhere upon completing charging.
      }

      _laser_consumption = 0.0;
      _idle_consumption = 0.0;

    }

    void system_power_consumption()
    {
      auto curr_time_pointer = std::chrono::system_clock::now();

      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;


      if(elapsed_seconds >= 1)
      {
        _laser_consumption+= LASER_POW_PS;
        _idle_consumption+= IDLE_POW_PS;

        _window_start = current_time;
      }

    }
  
  private:
      float _cumulative_consumption;
      float _idle_consumption;
      float _laser_consumption;
      int _window_length;
      int _window_start;
      
      const double MIN_CHARGE_THRESHOLD = 100.0;
      const float IDLE_POW_PS = 1.14; //as caused by the rebet software running. Idle consumption of the robot as a whole is considered a constant factor.
      const float LASER_POW_PS = 2.34; //watts

};


class SystemPowerQR : public SystemLevelQR
{
  public:

    SystemPowerQR(const std::string& name, const NodeConfig& config) : SystemLevelQR(name, config, QualityAttribute::Power)
    {
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      _idle_consumption = 0.0;
      _laser_consumption = 0.0; 

      _higher_is_better = false;

    }


    void initialize(float max_pics_ps, int window_length)
    {
        _window_length = window_length;

          // _max_idle_consumption = IDLE_POW * (_window_length + 1);
          // _max_laser_consumption = LASER_POW * (_window_length + 1);

    }


    static PortsList providedPorts()
    {
      PortsList base_ports = SystemLevelQR::providedPorts();

      PortsList child_ports = { 
              };

      child_ports.merge(base_ports);

      return child_ports;
    }

    
	
    virtual void calculate_measure() override
    {
      //As this is called on success, we assume for this case that child metrics only publish their values on their own respective successes.
      gather_child_metrics();
      
      double cumulative_task_consumptions = 0.0;

      for (const auto& [key, power_metric] : _sub_qr_metrics)
      {
        std::cout << "key: " <<  key << "metric: " << power_metric << std::endl;
        cumulative_task_consumptions+=power_metric;
      }


      _metric = cumulative_task_consumptions + _laser_consumption + _idle_consumption;

      output_metric();
      metric_mean();

      setOutput(MEAN_METRIC,_average_metric);
      
      if(_metric > MIN_CHARGE_THRESHOLD)
      {
        setOutput(QR_STATE,"below_min"); //it is set back to OK in the BT elsewhere upon completing charging.
      }


      _laser_consumption = 0.0;
      _idle_consumption = 0.0;

    }

    void system_power_consumption()
    {
      auto curr_time_pointer = std::chrono::system_clock::now();

      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;

      //std::cout << "elapsed seconds and window length inside powerQR " << elapsed_seconds << " " << _window_length << std::endl;


      if(elapsed_seconds >= 1)
      {
        _laser_consumption+= LASER_POW_PS;
        _idle_consumption+=  IDLE_POW_PS;

        _window_start = current_time;
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
          system_power_consumption();
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
      float _idle_consumption;
      float _laser_consumption;
      int _window_length;
      int _window_start;
      const double MIN_CHARGE_THRESHOLD = 100.0;
      const float IDLE_POW_PS = 1.14; //as caused by the rebet software running. Idle consumption of the robot as a whole is considered a constant factor.
      const float LASER_POW_PS = 2.34; //watts

};

class MovementEfficiencyQR : public TaskLevelQR
{
  public:
    builtin_interfaces::msg::Time last_timestamp;
    int window_start;

    MovementEfficiencyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::MovementEfficiency)
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
      PortsList base_ports = TaskLevelQR::providedPorts();

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

          if(_linear_speed > 0.03) //this is to circumvent a start from stationary being considered as part of this QR.
          {
            float speed_ratio = (_linear_speed/WAFFLE_MAX_LIN_VEL);
            _metric = std::clamp(speed_ratio,0.0f,1.0f);

            output_metric();
            metric_mean();

            setOutput(MEAN_METRIC,_average_metric);
          }
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

class SafetyQR : public TaskLevelQR
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
    SafetyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Safety)
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
      PortsList base_ports = TaskLevelQR::providedPorts();

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

          std::cout << "fitted nearest " << _fitted_nearest << std::endl;

          _metric = std::clamp(_fitted_nearest,0.0f,1.0f);

          std::cout << "metric in safety " << _metric << std::endl; 

          auto curr_time_pointer = std::chrono::system_clock::now();

          int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();

          output_metric();
          metric_mean();

          setOutput(MEAN_METRIC,_average_metric);
        }

      }


      
     


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
