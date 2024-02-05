using namespace BT;


// Example of custom SyncActionNode (synchronous action)
// without ports.


#include <iostream>
#include <vector>
#include <algorithm>
#include <math.h>
#include "rebet/adapt_node.h"



class AdaptSpiralRate : public OnRunningAdapt
{
  public:

    const std::string PICTURE_RT_PARAM = "pic_rate";
    const std::string ACTION_SRVR = "identify_action_server"; //now unnecessary, should be removed..

    AdaptSpiralRate(const std::string& name, const NodeConfig& config) : OnRunningAdapt(name, config)
    {
      std::vector<double> spiral_rate_values{1.0, 2.0, 3.0};
      registerAdaptations<double>(spiral_rate_values, PICTURE_RT_PARAM, ACTION_SRVR);

      //If you overwrite tick, and do this at different moments you can change the adaptation options at runtime.
      setOutput(VARIABLE_PARAMS, _var_params); 
  
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = OnRunningAdapt::providedPorts();

      PortsList child_ports =  {
              };
      child_ports.merge(base_ports);

      return child_ports;
    }


};