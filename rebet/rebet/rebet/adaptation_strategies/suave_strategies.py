#!/usr/bin/env python3
import numpy as np

from rebet.adaptation_strategies.adaptation_strategy import AdaptationStrategy
from lifecycle_msgs.msg import Transition


class ThrusterRecovery(AdaptationStrategy):

    def __init__(self):
        super().__init__('random')
        
    def suggest_adaptation(self, adaptation_state):
        
        thruster_condition = None
        transitions = []
        for qr_value in adaptation_state.qr_values:
            if(qr_value.name == "SearchEfficiencyQR"):
                thruster_condition = qr_value.qr_fulfilment

        if(thruster_condition == 0):
            t = Transition()
            t.id = 3 #Activate
            transitions.append(t)


        return t

        # if(keyvalue_msg.value == "FALSE")
        # {
        #     std::cout << "\n\n\n\n\n\nCondition MET Because of failure!!\n\n\n\n\n" << std::endl;

        #     _transitions = {};
        #     lifecycle_msgs::msg::Transition transition;
        #     transition.id = 3; //Activate transition
        #     this->_transitions.push_back(transition);
        #     return true; //Condition met!
        # }
        # else if(keyvalue_msg.value == "RECOVERED")
        # {
        #     //The way SUAVE works is that it recovers all thrusters no matter how many broke, so this condition is only met when for each thruster I hear recovered once.
        #     recovery_count++;
        #     std::cout << "recovery count " << recovery_count;
        #     if(recovery_count == 5)
        #     {
        #     recovery_count = 0;
        #     std::cout << "\n\n\n\n\n\nCondition MET Because of recovered!!\n\n\n\n\n" << std::endl;

        #     this->_transitions = {};
        #     lifecycle_msgs::msg::Transition transition;
        #     transition.id = 4; //Deactivate transition
        #     this->_transitions.push_back(transition);
            
        #     return true; //Condition met!
        #     }



suave_strategies = {
    'lifecycle_thruster_recovery': ThrusterRecovery
}
    

