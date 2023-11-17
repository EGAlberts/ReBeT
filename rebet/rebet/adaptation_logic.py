#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from rebet_msgs.msg import AdaptationState
from rebet_msgs.srv import AdaptSystem
from std_msgs.msg import Float64
import sys
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rcl_interfaces.msg import Parameter, ParameterValue
from abc import ABC, abstractmethod

class AdaptationLogic(Node, ABC):
    def __init__(self, node_name):
        super().__init__(node_name)
        
        # self.adapt_state_subscription = self.create_subscription(
        #     AdaptationState,
        #     'system_adaptation_state',
        #     self.adaptation_state_callback,
        #     10, 
        #     callback_group=MutuallyExclusiveCallbackGroup())
        self.adapt_system_service = self.create_service(
            AdaptSystem,
            '/adapt_system',
            self.adaptation_state_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        
    @abstractmethod
    def adaptation_state_callback():
        pass
    
