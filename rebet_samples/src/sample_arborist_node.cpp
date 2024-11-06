//This is just the sample_bt_executor from BT.ROS2 but with the Arborist as its parent instead.

#include "rebet/arborist.hpp"
#include "behaviortree_cpp/actions/sleep_node.h"

class SleepAndReport : public SleepNode
{
  public:
    SleepAndReport(const std::string& name, const NodeConfig& config)  : SleepNode(name,config)
    {
      _sleep_count = 0;
    }


    static PortsList providedPorts()
    {
      PortsList base_ports = SleepNode::providedPorts();

      PortsList child_ports =  {OutputPort<int>("sleep_count","How long the system has slept, in seconds"),};
      child_ports.merge(base_ports);

      return child_ports;
    }

    NodeStatus onRunning() override
    {
      _sleep_count+=1;
      setOutput("sleep_count",_sleep_count);
      return SleepNode::onRunning();
    }
  
  private:
    int _sleep_count;
};

class DontSleepTooLong : public TaskLevelQR
{
  public:
    DontSleepTooLong(const std::string& name, const NodeConfig& config) : TaskLevelQR(name, config, QualityAttribute::Test, NodeStatus::RUNNING)
    { }

    static PortsList providedPorts()
    {
      PortsList base_ports = TaskLevelQR::providedPorts();

      PortsList child_ports =  {InputPort<int>("sleep_count","How long the system has slept, in seconds"),};
      child_ports.merge(base_ports);

      return child_ports;
    }

    virtual void calculate_measure(NodeStatus status) override
    {
      if(status == NodeStatus::RUNNING)
      {
        int sleep_count;
        auto res = getInput("sleep_count", sleep_count);

        if(res)
        {
          _metric = sleep_count/sleep_threshold;

          if(sleep_count < sleep_threshold)
          {
            std::cout << "get rest" << std::endl;
            std::cout << sleep_count << std::endl;

            setOutput(QR_STATUS,"Get some more rest.");
          }
          else
          {
            setOutput(QR_STATUS,"Wake up!");
          }
        }
        else
        {
          std::cout << "not got sleep count" << std::endl;
        }
      }
    }

  private:
    int sleep_threshold = 2000;

};


class SampleArborist : public Arborist
{
public:
  SampleArborist(const rclcpp::NodeOptions& options) : Arborist(options)
  {

  }

   void registerNodesIntoFactory(BT::BehaviorTreeFactory& factory) override
  {      
    factory.registerNodeType<DontSleepTooLong>("DontSleepTooLong");
    factory.registerNodeType<SleepAndReport>("Sleep2");
  }



private:
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto sample_arborist = std::make_shared<SampleArborist>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
                                                std::chrono::milliseconds(250));
  exec.add_node(sample_arborist->node());
  exec.spin();
  exec.remove_node(sample_arborist->node());

  rclcpp::shutdown();
}