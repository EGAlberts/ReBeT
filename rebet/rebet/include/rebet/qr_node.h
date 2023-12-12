#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
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

enum class QualityAttribute {Power, Safety, TaskEfficiency, MovementEfficiency};


class QRNode : public DecoratorNode
{
public:

  QRNode(const std::string& name, const NodeConfig& config, QualityAttribute quality_attribute) : DecoratorNode(name, config)
  {
    _quality_attribute = quality_attribute;
    _average_metric = 0.0;
    _times_calculated = 0;
  }

  static PortsList providedPorts()
  {
      return {InputPort<double>(WEIGHT, "How much influence this QR should have in the calculation of system utility"), 
              OutputPort<double>(METRIC, "To what extent is this property fulfilled"),
              OutputPort<double>(MEAN_METRIC, "To what extent is this property fulfilled on average"),
              OutputPort<std::string>(QR_STATE, "Information as to the state the QR is currently in."),
              };
  }




  virtual ~QRNode() override = default;

  //Name for the weight input port
  static constexpr const char* WEIGHT = "weight";

  //Names for the metric output ports
  static constexpr const char* METRIC = "metric";
  static constexpr const char* MEAN_METRIC = "mean_metric";
  static constexpr const char* QR_STATE = "out_state";

  QualityAttribute qa_type()
  {
    return _quality_attribute;
  }

  double current_metric()
  {
    return _metric;
  }

  bool is_higher_better()
  {
    return _higher_is_better;
  }


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



  



  //void halt() override;

  protected:
    QualityAttribute _quality_attribute;
    int _times_calculated;
    double _average_metric;
    double _metric;
    bool _higher_is_better = true;


    void metric_mean()
    {
      double new_average = _average_metric + ((_metric - _average_metric) / (double)_times_calculated);
      _average_metric = new_average;
    }

    void output_metric()
    {
      setOutput(METRIC,_metric);
      _times_calculated++; 
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


};


class TaskLevelQR : public QRNode
{
  public:
    TaskLevelQR(const std::string& name, const NodeConfig& config, QualityAttribute quality_attribute) : QRNode(name, config, quality_attribute)
    {
    }

};
class SystemLevelQR : public QRNode
{
  public:
    SystemLevelQR(const std::string& name, const NodeConfig& config, QualityAttribute quality_attribute) : QRNode(name, config, quality_attribute)
    {
    }

   

 
  protected:
    void gather_child_metrics()
      {

        auto node_visitor = [this](TreeNode* node)
        {
          if (auto task_qr_node = dynamic_cast<TaskLevelQR*>(node))
          {
            if(_quality_attribute == task_qr_node->qa_type()) //If child TaskQR QA matches SystemLevelQR's QA.
            {
              double task_qr_metric = task_qr_node->current_metric();
              std::string task_qr_name = task_qr_node->registrationName();
              _sub_qr_metrics[task_qr_name] = task_qr_metric;
            }
          }

        };

        BT::applyRecursiveVisitor(child_node_, node_visitor);
      }
  protected:
      std::map<std::string, double> _sub_qr_metrics;

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
};







class SearchEfficiencyQR : public TaskLevelQR
{
  public:
    SearchEfficiencyQR(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::TaskEfficiency)
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
              InputPort<rebet::SystemAttributeValue>(STATUS_ONE, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_TWO, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_THREE, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_FOUR, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_FIVE, "status of thruster"),
              InputPort<rebet::SystemAttributeValue>(STATUS_SIX, "status of thruster"),
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

    void check_thrusters()
    {
      int recovery_count = 0; 
      for(unsigned int i = 0; i < thruster_stati.size(); i++)
      {
        auto status_portname = thruster_stati[i];

      
        rebet::SystemAttributeValue thruster_status_att; 
        auto res = getInput(status_portname, thruster_status_att);

        if(res)
        {
          auto as_sysatt_msg = thruster_status_att.to_value_msg();
          if(as_sysatt_msg.header.stamp != thruster_stati_timestamps[i])
          {
            thruster_stati_timestamps[i] = as_sysatt_msg.header.stamp;
            diagnostic_msgs::msg::KeyValue keyvalue_msg = thruster_status_att.get<rebet::SystemAttributeType::ATTRIBUTE_DIAG>();
            
            if(keyvalue_msg.value == "FALSE")
            {
              std::cout << "\n\n\n\n\n\nCondition MET Because of failure!!\n\n\n\n\n" << std::endl;
              setOutput(QR_STATE, "Thruster Failure");
            }
            else if(keyvalue_msg.value == "RECOVERED")
            {
              //The way SUAVE works is that it recovers all thrusters no matter how many broke, so this condition is only met when for each thruster I hear recovered once.
              recovery_count++;
              std::cout << "recovery count " << recovery_count;
              if(recovery_count == 5)
              {
                recovery_count = 0;
                std::cout << "\n\n\n\n\n\nCondition MET Because of recovered!!\n\n\n\n\n" << std::endl;
                setOutput(QR_STATE, "Thrusters Recovered");              
              }
            }
          }
        }

      }
    }

    virtual void calculate_measure() override
    {

      
      setOutput(QR_STATE, "OK");
      check_thrusters();

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
      static constexpr const char* STATUS_ONE = "thruster_status_one";
      static constexpr const char* STATUS_TWO = "thruster_status_two";
      static constexpr const char* STATUS_THREE = "thruster_status_three";
      static constexpr const char* STATUS_FOUR = "thruster_status_four";
      static constexpr const char* STATUS_FIVE = "thruster_status_five";
      static constexpr const char* STATUS_SIX = "thruster_status_six";

      std::vector<std::string> thruster_stati = {STATUS_ONE, STATUS_TWO, STATUS_THREE, STATUS_FOUR, STATUS_FIVE, STATUS_SIX};
      std::vector<builtin_interfaces::msg::Time> thruster_stati_timestamps = {builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time(), builtin_interfaces::msg::Time()};
};




}   // namespace BT
