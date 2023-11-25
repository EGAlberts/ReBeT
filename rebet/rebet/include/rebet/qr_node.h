#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rebet_msgs/msg/objects_identified.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"

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
class QRNode : public DecoratorNode
{
public:

  QRNode(const std::string& name, const NodeConfig& config) : DecoratorNode(name, config)
  {
    _average_metric = 0.0;
    _times_calculated = 0;
  }

  static PortsList providedPorts()
  {
      return {InputPort<double>(WEIGHT, "How much influence this QR should have in the calculation of system utility"), 
              OutputPort<double>(METRIC, "To what extent is this property fulfilled"),
              OutputPort<double>(MEAN_METRIC, "To what extent is this property fulfilled on average")};
  }


  virtual ~QRNode() override = default;

  //Name for the weight input port
  static constexpr const char* WEIGHT = "weight";

  //Names for the metric output ports
  static constexpr const char* METRIC = "metric";
  static constexpr const char* MEAN_METRIC = "mean_metric";



private:
  int weight_;
  bool read_parameter_from_ports_;

  virtual NodeStatus tick() override
  {
    setStatus(NodeStatus::RUNNING);
    const NodeStatus child_status = child_node_->executeTick();

    switch (child_status)
    {
      case NodeStatus::SUCCESS: {
        resetChild();
        return NodeStatus::SUCCESS;
      }

      case NodeStatus::FAILURE: {
        resetChild();
        return NodeStatus::FAILURE;
      }

      case NodeStatus::RUNNING: {
        calculate_measure();
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

  virtual void calculate_measure()
  {
    getInput(WEIGHT, weight_);
    std::stringstream ss;

    ss << "Weight Port info received: ";
    // for (auto number : feedback->left_time) {
    ss << weight_;
    std::cout << ss.str().c_str() << std::endl;
    std::cout << "Here's where I would calculate a measure... if I had one" << std::endl;
  }


  //void halt() override;

  protected:
    int _times_calculated;
    double _average_metric;
    double _metric;

    void metric_mean()
    {
      double new_average = _average_metric + ((_metric - _average_metric) / (double)_times_calculated);
      _average_metric = new_average;
    }

    void output_metric()
    {
      setOutput(METRIC,std::min(_metric,1.0));
      _times_calculated++; 
    }


};

class TaskEfficiencyQR : public QRNode
{
  public:
    TaskEfficiencyQR(const std::string& name, const NodeConfig& config) : QRNode(name, config)
    {
      _counter = 0;

      _detected_in_window = 0.0;
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      _last_timestamp = builtin_interfaces::msg::Time();
  
    }

    void initialize(double max_objs_ps, int window_length)
    {
        _max_object_ps = max_objs_ps;
        _window_length = window_length;

        _max_detected = _max_object_ps * double(_window_length); //What we presume is the max number of objects that'll be detected in a 20 second window.
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
      rebet_msgs::msg::ObjectsIdentified objects_msg;

      getInput(IN_OBJ, objects_msg);

      _counter += 1;
      if((objects_msg.object_detected == true) && (objects_msg.stamp != _last_timestamp))
      {
        _detected_in_window += objects_msg.object_names.size();
        _last_timestamp = objects_msg.stamp;
      }

      _metric = (double)_detected_in_window / _max_detected;

      auto curr_time_pointer = std::chrono::system_clock::now();
      
      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;
      if(elapsed_seconds >= _window_length)
      {
        output_metric();
        metric_mean();
        setOutput(MEAN_METRIC,_average_metric);
        _window_start = current_time;
        _detected_in_window = 0;
      }

    }

    private:
      int _detected_in_window;
      builtin_interfaces::msg::Time _last_timestamp;
      double _max_detected;
      double _max_object_ps;
      int _window_length;
      int _window_start;
      int _counter;
      static constexpr const char* IN_OBJ = "objs_identified";

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
        _max_picture_consumption = (max_pics_ps * (_window_length+1)) * detection_average_power;
        _max_idle_consumption = _idle_power * (_window_length + 1);
        _max_laser_consumption = _laser_power * (_window_length + 1);

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
      getInput(IN_PIC_RATE,_pic_rate);

      getInput(IN_OBJ, _objects_msg);

      if(res)
      {
        _odom_msg = _odom_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_ODOM>();
        _linear_speed = hypot(fabs(_odom_msg.twist.twist.linear.x), fabs(_odom_msg.twist.twist.linear.y));
      }

      if(_objects_msg.stamp != _obj_last_timestamp)
      {
        _pictures_taken_in_window += _pic_rate;
        _obj_last_timestamp = _objects_msg.stamp;
      }


      if( (_odom_msg.header.stamp.sec-_odom_last_timestamp_sec) >= 1 ) //one second has passed since last odom
      {
        
        _motion_consumption+= calculate_power_motion(_linear_speed);
        _laser_consumption+= _laser_power;
        _idle_consumption+=  _idle_power;

        _odom_last_timestamp_sec = _odom_msg.header.stamp.sec;
        
      }



      auto curr_time_pointer = std::chrono::system_clock::now();

      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;

      //std::cout << "elapsed seconds and window length inside powerQR " << elapsed_seconds << " " << _window_length << std::endl;


      if(elapsed_seconds >= _window_length)
      {
        float picture_consumption = _pictures_taken_in_window * detection_average_power;
        float total_consumption = _motion_consumption + picture_consumption + _idle_consumption +  _laser_consumption;

        //std::cout << total_consumption << " is ttcnsmp " <<  _max_consumption << " is max consumption" << std::endl;

        _metric = 1 - (total_consumption/_max_consumption); //1 - because power consumption is a bad thing for the QA

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


      const float _idle_power = 1.14; //as caused by the detection software running. Idle consumption of the robot as a whole is considered a constant factor.
      const float detection_average_power = 10.919800; //watts
      const float _laser_power = 2.34; //watts
      const float WAFFLE_MAX_LIN_VEL = 0.26;


      static constexpr const char* IN_PIC_RATE = "in_picture_rate";
      static constexpr const char* IN_ODOM = "in_odom";
      static constexpr const char* IN_OBJ = "objs_identified";






};

class SearchEfficiencyQR : public QRNode
{
  public:
    SearchEfficiencyQR(const std::string& name, const NodeConfig& config) : QRNode(name, config)
    {
      _window_start = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();

      _last_timestamp = builtin_interfaces::msg::Time();

      _water_visibilities = {};
  
    }

    void initialize(int window_length)
    {
        _window_length = window_length;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = QRNode::providedPorts();

      PortsList child_ports =  {
              InputPort<rebet::SystemAttributeValue>(IN_WATER,"water_visibility message wrapped in keyvalue message wrapped in a systemattributevalue instance"),

              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure() override
    {
      auto res = getInput(IN_WATER,_watervis_attribute); 

      if(res)
      {
        auto as_sysatt_msg = _watervis_attribute.to_value_msg();
        if(as_sysatt_msg.header.stamp != _last_timestamp)
        {
          _keyvalue_msg = _watervis_attribute.get<rebet::SystemAttributeType::ATTRIBUTE_DIAG>();
          float water_visibility = std::stof(_keyvalue_msg.value);

          _water_visibilities.push_back(water_visibility);

          _last_timestamp = as_sysatt_msg.header.stamp;

          std::cout << "new water vis received " << water_visibility; 
        }
      }

      

      auto curr_time_pointer = std::chrono::system_clock::now();
      
      int current_time = std::chrono::duration_cast<std::chrono::seconds>(curr_time_pointer.time_since_epoch()).count();
      int elapsed_seconds = current_time-_window_start;
      if(elapsed_seconds >= _window_length)
      {
        float wvis_sum = std::accumulate(_water_visibilities.begin(), _water_visibilities.end(), 0);
        float average_water_vis = wvis_sum / (float)_water_visibilities.size();
        _metric = (average_water_vis - _min_wvis) / (_max_wvis - _min_wvis);

        output_metric();
        metric_mean();
        setOutput(MEAN_METRIC,_average_metric);
        _window_start = current_time;
        _water_visibilities = {};
      }

    }

    private:
      rebet::SystemAttributeValue _watervis_attribute;
      diagnostic_msgs::msg::KeyValue _keyvalue_msg;


      std::vector<float> _water_visibilities;
      int _detected_in_window;
      builtin_interfaces::msg::Time _last_timestamp;
      double _max_detected;
      double _max_object_ps;
      int _window_length;
      int _window_start;
      int _coun_max_wvister;
      float _max_wvis = 3.75; //This is a parameter for suave.. I'd say it should be linked, but ideally I'd just dynamically bound it instead.
      float _min_wvis = 1.25; //idem ditto


      static constexpr const char* IN_WATER = "in_water_visibility";

};




}   // namespace BT
