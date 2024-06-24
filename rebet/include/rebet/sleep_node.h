#include "behaviortree_cpp/behavior_tree.h"

//This is one of the sample nodes of BT.CPP which has some use for me.
class SleepNode : public BT::StatefulActionNode
{
  public:
    SleepNode(const std::string& name, const BT::NodeConfig& config)
      : BT::StatefulActionNode(name, config)
    {}

    static BT::PortsList providedPorts()
    {
        // amount of milliseconds that we want to sleep
        return{ BT::InputPort<int>("msec") };
    }

    BT::NodeStatus onStart() override
    {
        int msec = 0;
        getInput("msec", msec);
        if( msec <= 0 )
        {
            // no need to go into the RUNNING state
            return  BT::NodeStatus::SUCCESS;
        }
        else {
            using namespace std::chrono;
            // once the deadline is reached, we will return SUCCESS.
            deadline_ = system_clock::now() + milliseconds(msec);
            return  BT::NodeStatus::RUNNING;
        }
    }

    /// method invoked by an action in the RUNNING state.
     BT::NodeStatus onRunning() override
    {
        if ( std::chrono::system_clock::now() >= deadline_ )
        {
            return BT::NodeStatus::SUCCESS;
        }
        else {
            return  BT::NodeStatus::RUNNING;
        }
    }

    void onHalted() override
    {
        // nothing to do here...
        std::cout << "SleepNode interrupted" << std::endl;
    }

  private:
    std::chrono::system_clock::time_point deadline_;
};