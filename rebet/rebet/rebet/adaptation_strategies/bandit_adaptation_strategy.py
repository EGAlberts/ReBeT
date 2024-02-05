#!/usr/bin/env python3
import time

from masced_bandits.bandit_options import initialize_arguments, bandit_args
from masced_bandits.bandits import init_bandit
from masced_bandits.utilities import truncate

import sys

import csv
import os
import numpy as np
from rebet.adaptation_strategies.adaptation_strategy import AdaptationStrategy

HYPERPARAM_PARAM = "hyperparameters"
BANDITNAME_PARAM = "bandit_name"

session_names = [
    "Quantum",
    "Nebula",
    "Odyssey",
    "Cipher",
    "Zenith",
    "Synapse",
    "Apex",
    "Cosmos",
    "Genesis",
    "Spectrum",
    "Infinity",
    "Aegis",
    "Velocity",
    "Orion",
    "Phoenix",
    "Catalyst",
    "Elysium",
    "Helix",
    "Serenity",
    "Pinnacle",
    "Eclipse",
    "Pandora",
    "Polaris",
    "Nova",
    "Aether",
    "Metropolis",
    "Aurora",
    "Equinox",
    "Radiance",
    "Celestial",
    "Paragon",
    "Quasar",
    "Epoch",
    "Luminary",
    "QuantumCore",
    "Nebulous",
    "Ascendant",
    "Zenithal",
    "Heliosphere",
    "Astrolabe",
    "Empyrean",
    "Utopia",
    "Ethereal",
    "Synthwave",
    "Solstice",
    "Stardust",
    "Apexia",
    "Temporal",
    "AstraNova",
    "Perigee"
]




chosen_arms = []





class BanditStrategy(AdaptationStrategy):

    def __init__(self, bandit_name):
        super().__init__(bandit_name.lower())
        print("\n\nbandit created!\n\n")
        self.round_num = 0
        self.bandit_name = bandit_name
        # self.declare_parameter(HYPERPARAM_PARAM,[""])
        # self.hyper_parameters = self.get_parameter(HYPERPARAM_PARAM).get_parameter_value().string_array_value
        self.hyper_param_kwargs = {}

        # if self.hyper_parameters != [] and self.hyper_parameters != [""]:
        #     for i in range(0,len(self.hyper_parameters),2):
        #         self.hyper_param_kwargs[self.hyper_parameters[i]] = self.hyper_parameters[i+1]
        
        csv_file = 'bandit_report.csv'
        file = open(csv_file, mode='a', newline='')
        self.writer = csv.writer(file)

        file_exists = os.path.isfile(csv_file)

        if not file_exists:
            self.writer.writerow(['Session_Name', 'Timestamp', 'Adaptation Chosen', 'Reward', 'Bounds']) 

        file.close()

        bandit_args["dynamic_bounds"] = True
        bandit_args["bounds"] = [0,0.000000000001]


        self.configuration_dict = {}
        self.prev_configurations_msg = None
 
        self.utilities = []

        self.bandit_not_initialized = True

        self.session_name = np.random.choice(session_names)



    def initialize_bandit(self):
        self.make_hashable()
        bandit_args["arms"] = list(self.configuration_dict.keys())
        bandit_args["initial_configuration"] = bandit_args["arms"][0]
        self.bandit_instance = init_bandit(name=self.bandit_name, **self.hyper_param_kwargs)

        self.prev_configurations_msg = self.possible_configs
        # self.get_logger().info("epsi " + str(self.bandit_instance.epsilon))


   

    def make_hashable(self):
        for config_msg in self.possible_configs:
            string_repr = " ".join([str((param.name, param.value)) for param in config_msg.configuration_parameters] + config_msg.node_names + [str(adap_type) for adap_type in config_msg.adaptation_target_types])
            if string_repr not in self.configuration_dict:
                self.configuration_dict[string_repr] = config_msg
        

    def suggest_adaptation(self, adaptation_state):
        self.possible_configs = adaptation_state.possible_configurations

        if(self.bandit_not_initialized):
            self.initialize_bandit()
            self.bandit_not_initialized = False


        if((self.prev_configurations_msg != self.possible_configs)):
            #New configurations have been added 
            self.make_hashable()

            self.prev_configurations_msg = adaptation_state.possible_configurations
            
            self.bandit_instance.arms = list(self.configuration_dict.keys()) #The chosen bandit might not properly support the change in arms.
            


        #since python 3.7 the order given by .keys() is guaranteed to be insertion order, this means any new arms should be added added to a len+1 index in the dictionary and not get confused for others.


        reward = 0 
        #for qr in msg.qr_values: self.reward+=qr.qr_fulfilment 
        self.utilities = [adap_utility for adap_utility in adaptation_state.current_utility ]
       
        reward = np.mean(self.utilities) #should be made more nuanced
        print("\n\n\n REWARD RECEIVED   \n\n\n" + str(reward))
        # reward, _, _ = truncate(reward)
        next_arm = self.bandit_instance.get_next_arm(reward)
        self.round_num+=1
        file = open('bandit_report.csv', mode='a', newline='')
        writer = csv.writer(file)
        row = [self.session_name, str(time.time()), str(next_arm), str(reward), str(bandit_args["bounds"]), str(self.utilities), str(self.bandit_name), str(self.hyper_param_kwargs), str(self.round_num)]
        writer.writerow(row)
        file.close()
        
        return self.configuration_dict[next_arm]
        