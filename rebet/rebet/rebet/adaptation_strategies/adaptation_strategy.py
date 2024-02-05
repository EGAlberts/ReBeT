#!/usr/bin/env python3

import time


from rebet_msgs.msg import AdaptationState

import sys

from abc import ABC, abstractmethod

class AdaptationStrategy(ABC):
    def __init__(self,strategy_name):
        self.name = strategy_name + "_strategy"
        
    def get_name(self):
        return self.name
    
    @abstractmethod
    def suggest_adaptation(adaptation_state):
        pass
    
