#!/usr/bin/env python3
import numpy as np

from rebet.rebet.adaptation_strategies.adaptation_strategy import AdaptationStrategy

class RandomStrategy(AdaptationStrategy):

    def __init__(self):
        super().__init__('random_adaptation_strategy')
        
    def suggest_adaptation(adaptation_state):
        possible_configs = adaptation_state.system_possible_configurations

        chosen_config = np.random.choice(possible_configs)

        return chosen_config