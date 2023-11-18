#!/usr/bin/env python3
import numpy as np

from rebet.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class RandomStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('random')
        
    def suggest_adaptation(self, adaptation_state):
        possible_configs = adaptation_state.system_possible_configurations

        chosen_config = np.random.choice(possible_configs)

        return chosen_config