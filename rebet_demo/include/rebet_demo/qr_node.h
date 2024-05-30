#pragma once

#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/behavior_tree.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rebet_msgs/msg/objects_identified.hpp"
#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet_demo/system_attribute_value.hpp"

namespace BT
{
/**
 * @brief The QRNode is used to constrain variable action nodes it decorates.
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
      return {InputPort<double>(WEIGHT, 1.0, "How much influence this QR should have in the calculation of system utility"), 
              OutputPort<double>(METRIC, "To what extent is this property fulfilled"),
              OutputPort<double>(MEAN_METRIC, "To what extent is this property fulfilled on average"),
              OutputPort<std::string>(QR_STATUS, "Information as to the status the QR currently has."),
              };
  }




  virtual ~QRNode() override = default;

  //Name for the weight input port
  static constexpr const char* WEIGHT = "weight";

  //Names for the metric output ports
  static constexpr const char* METRIC = "metric";
  static constexpr const char* MEAN_METRIC = "mean_metric";
  static constexpr const char* QR_STATUS = "status";

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

    virtual void calculate_measure() {}

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

}   // namespace BT
